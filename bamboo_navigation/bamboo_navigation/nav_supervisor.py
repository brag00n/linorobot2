#!/usr/bin/env python3
"""nav_supervisor -- les SOUS-MODES de la navigation, les cartes, les points, le diagnostic.

Le mode NAV du robot a deux sous-modes (demande utilisateur 2026-10-10) :
  exploration  CREER une carte : slam_toolbox en cartographie (il publie map et map->odom).
               On conduit a la manette ; Nav2 est actif (on peut aussi lui donner un but).
  navigation   NAVIGUER sur une carte enregistree : map_server + AMCL (cycle de vie Nav2),
               puis buts par POINTS de navigation (plus tard : recherche par la camera).
  idle         rien ne cartographie ni ne localise ; Nav2 au repos.
Un seul sous-mode a la fois : slam_toolbox et AMCL publient tous deux map->odom.

POURQUOI UN SUPERVISEUR et pas deux launch : slam_toolbox (Humble) n'est pas un noeud a cycle
de vie, AMCL et map_server le sont. Passer de l'un a l'autre sans tout relancer demande de
DEMARRER/ARRETER slam_toolbox (sous-processus) et de piloter les gestionnaires de cycle de vie
de Nav2 (localisation, navigation). C'est ce noeud.

UNE SEULE PORTE D'ENTREE : le service ~/command (NavCommand), pour la manette comme pour le
MCP. L'etat part sur /navigation/state (NavState, latche).

DIAGNOSTIC `navigation` DEPENDANT DES CAPTEURS : la liste des capteurs EXIGES vient du bloc
`sensors:` du robot (nav_params.required_sensors). Un capteur absent du robot n'est pas exige ;
sans lidar, la navigation est declaree impossible (ERROR), avec la raison.
"""
import math
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from tf2_ros import Buffer, TransformListener

from bamboo_interfaces.msg import NavState, RobotMode
from bamboo_interfaces.srv import NavCommand
from bamboo_navigation import frontier, map_io

STARTUP, RESET = ManageLifecycleNodes.Request.STARTUP, ManageLifecycleNodes.Request.RESET


class NavSupervisor(Node):

    def __init__(self):
        super().__init__("nav_supervisor")
        self._cbg = ReentrantCallbackGroup()
        d = self.declare_parameter
        self.maps_dir = str(d("maps_dir", "/tmp/maps").value)
        self.slam_params = str(d("slam_params_file", "").value)
        self.default_map = str(d("default_map", "").value)
        self.required = [s for s in d("required_sensors", ["lidar"]).value if s]
        self.scan_topic = str(d("scan_topic", "/scan").value)
        self.imu_topic = str(d("imu_topic", "/imu/data").value)
        self.odom_topic = str(d("odom_topic", "/odom").value)
        self.map_topic = str(d("map_topic", "/map").value)
        self.use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self.stale_s = float(d("sensor_stale_s", 2.0).value)
        self.loc_cov_warn = float(d("localization_cov_warn", 0.5).value)
        # --- exploration AUTOMATIQUE (frontieres) : reglages, cf. _autoTick ---
        self.auto_min_frontier_m = float(d("auto_min_frontier_m", 0.30).value)
        self.auto_goal_timeout_s = float(d("auto_goal_timeout_s", 60.0).value)
        self.auto_max_duration_s = float(d("auto_max_duration_s", 900.0).value)
        self.auto_resume_s = float(d("auto_resume_s", 3.0).value)
        self.auto_battery_min_v = float(d("auto_battery_min_v", 10.8).value)
        teleop_topic = str(d("teleop_topic", "/cmd_vel_teleop").value)
        battery_topic = str(d("battery_topic", "/battery").value)
        mode_topic = str(d("mode_topic", "/bamboo/mode").value)

        self.submode = "idle"
        self.map_name = ""
        self.slam = None              # sous-processus slam_toolbox
        self.goal_name, self.goal_status = "", ""
        self.distance, self.recoveries = math.nan, 0
        self.loc_cov = math.nan
        self.detail = "pret"
        self._goal = None
        self._goalSeq = 0
        self._cancelPending = False
        self._next = 0
        self._lock = threading.Lock()
        self._last = {"scan": None, "imu": None, "odom": None}
        # Etat de l'exploration automatique : off | running | paused | homing | done.
        self.auto = "off"
        self._autoResumeTo = "running"   # etat a reprendre apres une pause manette
        self._autoT0 = 0.0
        self._autoHome = None            # pose de depart (map), prise au premier tour
        self._autoTarget = None          # (x, y) du but en cours
        self._autoGoalT = 0.0
        self._autoOwnCancel = False      # annulation par NOUS (pause) : pas une liste noire
        self._autoAvoid = []             # points deja tentes (echecs et buts atteints)
        self._autoN = 0
        self._autoNoFront = 0
        self._autoHomeTries = 0
        self._autoFronts = 0
        self._autoKnown = math.nan
        self._teleopT = 0.0              # dernier ordre NON NUL de la manette
        self._battV = math.nan
        self._robotMode = ""

        # TF ecoutee SEULEMENT hors idle (cf. _tfListen) : un TransformListener Python sur tout
        # /tf, sous MultiThreadedExecutor, coutait a lui seul une bonne part des ~30 % d'un
        # coeur que prenait le superviseur au repos (essai Nb, 2026-10-10).
        self.tf = Buffer()
        self.tfl = None
        # Fraicheur seule : abonnements BRUTS (raw=True, sans deserialisation) portes par un
        # noeud SATELLITE tourne par un executeur MONO-fil dans son propre thread. Sous le
        # MultiThreadedExecutor du superviseur, chacun des ~50 messages/s reveillait le pool
        # de fils Python : ~30 % d'un coeur du RPi4 au repos (essai Nb, 2026-10-10).
        # use_global_arguments=False : sinon le `__node:=nav_supervisor` du launch le renomme
        # lui aussi, et le graphe montre deux /nav_supervisor (services de parametres en double).
        self._watch = rclpy.create_node("nav_supervisor_watch", use_global_arguments=False)
        self._watch.create_subscription(LaserScan, self.scan_topic,
                                        lambda _m: self._seen("scan"),
                                        qos_profile_sensor_data, raw=True)
        self._watch.create_subscription(Imu, self.imu_topic, lambda _m: self._seen("imu"),
                                        qos_profile_sensor_data, raw=True)
        self._watch.create_subscription(Odometry, self.odom_topic,
                                        lambda _m: self._seen("odom"), 1, raw=True)
        # Manette, batterie et mode du robot : pour la PAUSE et les garde-fous de l'exploration
        # automatique. Sur le satellite aussi : peu de messages, et rien a faire tourner sous
        # le verrou du superviseur.
        self._watch.create_subscription(Twist, teleop_topic, self._onTeleop, 5)
        self._watch.create_subscription(BatteryState, battery_topic,
                                        lambda m: setattr(self, "_battV", float(m.voltage)), 1)
        self._watch.create_subscription(
            RobotMode, mode_topic, lambda m: setattr(self, "_robotMode", m.mode),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._watchEx = SingleThreadedExecutor()
        self._watchEx.add_node(self._watch)
        threading.Thread(target=self._watchEx.spin, name="nav_watch", daemon=True).start()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # La carte est DEMANDEE a la sauvegarde (GetMap), pas suivie en continu : s'abonner a
        # /map faisait deserialiser toute la grille en Python a chaque mise a jour de
        # slam_toolbox (2 s), pour une carte qui ne sert qu'a `save_map` (2026-10-11).
        self.mapCli = {
            "exploration": self.create_client(GetMap, "/slam_toolbox/dynamic_map",
                                              callback_group=self._cbg),
            "navigation": self.create_client(GetMap, "/map_server/map",
                                             callback_group=self._cbg)}
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._onAmcl, 10,
                                 callback_group=self._cbg)
        self.pubState = self.create_publisher(NavState, "/navigation/state", latched)
        self.pubInit = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.locMgr = self.create_client(ManageLifecycleNodes,
                                         "/lifecycle_manager_localization/manage_nodes",
                                         callback_group=self._cbg)
        self.navMgr = self.create_client(ManageLifecycleNodes,
                                         "/lifecycle_manager_navigation/manage_nodes",
                                         callback_group=self._cbg)
        self.mapParams = self.create_client(SetParameters, "/map_server/set_parameters",
                                            callback_group=self._cbg)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose",
                                callback_group=self._cbg)
        self.create_service(NavCommand, "~/command", self._onCommand, callback_group=self._cbg)

        self.diag = Updater(self)
        self.diag.setHardwareID("navigation")
        self.diag.add("navigation", self._diagNav)
        self.create_timer(1.0, self._publish, callback_group=self._cbg)
        self.create_timer(1.0, self._autoTick, callback_group=self._cbg)
        os.makedirs(self.maps_dir, exist_ok=True)
        self.get_logger().info("superviseur de navigation : cartes dans %s, capteurs exiges %s"
                               % (self.maps_dir, self.required or "aucun"))

    # ------------------------------------------------------------ capteurs
    def _seen(self, k):
        self._last[k] = time.monotonic()

    def _onTeleop(self, m):
        if abs(m.linear.x) > 1e-3 or abs(m.angular.z) > 1e-3:
            self._teleopT = time.monotonic()

    def _onAmcl(self, m):
        c = m.pose.covariance
        self.loc_cov = float(c[0] + c[7])

    def _fresh(self, k):
        t = self._last.get(k)
        return t is not None and time.monotonic() - t <= self.stale_s

    def _tfListen(self, on):
        if on and self.tfl is None:
            self.tfl = TransformListener(self.tf, self)
        elif not on and self.tfl is not None:
            self.tfl.unregister()
            self.tfl = None
            self.tf.clear()

    def _tfOk(self, parent, child):
        if self.tfl is None:
            return False
        try:
            return self.tf.can_transform(parent, child, Time(),
                                         timeout=Duration(seconds=0.05))
        except Exception:                                   # noqa: BLE001
            return False

    def missing(self):
        """Capteurs/briques EXIGES absents ou muets, pour le sous-mode courant."""
        miss = []
        if "lidar" in self.required and not self._fresh("scan"):
            miss.append("lidar (%s muet)" % self.scan_topic)
        if "imu" in self.required and not self._fresh("imu"):
            miss.append("imu (%s muet)" % self.imu_topic)
        if not self._fresh("odom"):
            miss.append("odometrie (%s muette : EKF ?)" % self.odom_topic)
        # Au repos, la TF n'est pas ecoutee : l'EKF publie odom ET la TF, la fraicheur de
        # odom en tient lieu.
        if self.tfl is not None and not self._tfOk("odom", "base_footprint"):
            miss.append("TF odom->base_footprint")
        if self.submode in ("exploration", "navigation") and not self._tfOk("map", "odom"):
            miss.append("TF map->odom (%s)" % ("slam_toolbox" if self.submode == "exploration"
                                                else "AMCL non localise"))
        return miss

    # ------------------------------------------------------------ cycle de vie
    def _wait(self, fut, timeout):
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        return ev.wait(timeout) and fut.result() is not None

    def _manage(self, cli, cmd, timeout=30.0):
        if not cli.wait_for_service(timeout_sec=2.0):
            return False, "%s absent" % cli.srv_name
        fut = cli.call_async(ManageLifecycleNodes.Request(command=cmd))
        if not self._wait(fut, timeout):
            return False, "%s sans reponse" % cli.srv_name
        return bool(fut.result().success), ("ok" if fut.result().success else "refuse")

    def _startSlam(self):
        if self.slam is not None and self.slam.poll() is None:
            return True, "deja lance"
        cmd = ["ros2", "run", "slam_toolbox", "async_slam_toolbox_node", "--ros-args",
               "--params-file", self.slam_params, "-r", "map:=%s" % self.map_topic]
        self.slam = subprocess.Popen(cmd, start_new_session=True)
        t = time.monotonic()
        while time.monotonic() - t < 15.0:
            if self._tfOk("map", "odom"):
                return True, "slam_toolbox actif"
            if self.slam.poll() is not None:
                return False, "slam_toolbox s'est arrete (code %s)" % self.slam.returncode
            time.sleep(0.3)
        return True, "slam_toolbox lance (pas encore de map->odom : bouger le robot)"

    def _stopSlam(self):
        if self.slam is None or self.slam.poll() is not None:
            self.slam = None
            return
        try:
            os.killpg(self.slam.pid, signal.SIGINT)
            self.slam.wait(timeout=8.0)
        except Exception:                                   # noqa: BLE001
            try:
                os.killpg(self.slam.pid, signal.SIGKILL)
            except Exception:                               # noqa: BLE001
                pass
        self.slam = None

    def _setMap(self, name):
        path = os.path.join(self.maps_dir, name + ".yaml")
        if not os.path.isfile(path):
            return False, "carte %s introuvable" % path
        if not self.mapParams.wait_for_service(timeout_sec=2.0):
            return False, "map_server absent"
        req = SetParameters.Request(
            parameters=[Parameter("yaml_filename", value=path).to_parameter_msg()])
        fut = self.mapParams.call_async(req)
        if not self._wait(fut, 5.0):
            return False, "map_server sans reponse"
        return True, path

    def _initialPose(self):
        wps = map_io.load_waypoints(self.maps_dir, self.map_name)
        home = next((w for w in wps if w["name"] == "depart"), None) or \
            {"x": 0.0, "y": 0.0, "yaw": 0.0}
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(home["x"]), float(home["y"])
        m.pose.pose.orientation.z = math.sin(float(home["yaw"]) / 2.0)
        m.pose.pose.orientation.w = math.cos(float(home["yaw"]) / 2.0)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.2
        for _ in range(3):
            self.pubInit.publish(m)
            time.sleep(0.2)
        return "pose initiale : %s" % ("depart" if wps else "origine de la carte")

    # ------------------------------------------------------------ sous-modes
    def setSubmode(self, mode, arg=""):
        if mode == self.submode and mode != "navigation":
            return True, "deja en %s" % mode
        if mode != "exploration" and self.auto != "off":
            self._autoStop("sous-mode %s" % mode, nav_reset=False)
        self._cancelGoal()
        self._tfListen(mode in ("exploration", "navigation"))
        if mode == "exploration":
            self._manage(self.locMgr, RESET)
            ok, why = self._startSlam()
            if not ok:
                return False, why
            self.map_name = map_io.stamp_name("exploration")
            # Nav2 RESTE ARRETE en exploration : on y conduit a la manette, aucun but n'est
            # donne, et ses costmaps + son controleur coutaient ~50 % d'un coeur du RPi4 pour
            # rien -- assez pour affamer le lidar (scan juge muet) au demarrage (2026-10-11).
            self._manage(self.navMgr, RESET)
        elif mode == "navigation":
            maps = map_io.list_maps(self.maps_dir)
            name = arg or self.default_map or (maps[0] if maps else "")
            if not name:
                return False, "aucune carte : explorer puis sauvegarder d'abord"
            self._stopSlam()
            self._manage(self.locMgr, RESET)
            ok, why = self._setMap(name)
            if not ok:
                return False, why
            ok, why = self._manage(self.locMgr, STARTUP)
            if not ok:
                return False, "localisation : " + why
            self.map_name = name
            self._next = 0
            why = self._initialPose()
            self._manage(self.navMgr, STARTUP)
        elif mode == "idle":
            self._stopSlam()
            self._manage(self.locMgr, RESET)
            self._manage(self.navMgr, RESET)
            self.map_name = ""
        else:
            return False, "sous-mode inconnu : %s" % mode
        self.submode = mode
        self.detail = "sous-mode %s%s" % (mode, (" (%s)" % self.map_name) if self.map_name else "")
        self.get_logger().info(self.detail)
        return True, self.detail

    # ------------------------------------------------------------ buts
    def _robotPose(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time(),
                                         timeout=Duration(seconds=0.5))
        except Exception:                                   # noqa: BLE001
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                map_io.yaw_of(q.z, q.w, q.x, q.y))

    def _goto(self, wp):
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return False, "navigate_to_pose absent (Nav2 inactif ?)"
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(wp["x"]), float(wp["y"])
        p.pose.orientation.z = math.sin(float(wp["yaw"]) / 2.0)
        p.pose.orientation.w = math.cos(float(wp["yaw"]) / 2.0)
        goal.pose = p
        self.goal_name, self.goal_status = wp["name"], "active"
        self.distance, self.recoveries = math.nan, 0
        # Buts NUMEROTES : l'accuse et le resultat d'un but ANNULE peuvent arriver apres
        # l'envoi du suivant (exploration auto : pause manette, frontiere suivante) ; sans le
        # numero ils ecraseraient l'etat du but en cours.
        self._goalSeq += 1
        seq = self._goalSeq
        self._cancelPending = False
        fut = self.nav.send_goal_async(
            goal, feedback_callback=lambda fb, s=seq: self._onFeedback(fb, s))
        fut.add_done_callback(lambda f, s=seq: self._onAccepted(f, s))
        return True, "en route vers %s" % wp["name"]

    def _onFeedback(self, fb, seq):
        if seq != self._goalSeq:
            return
        f = fb.feedback
        self.distance = float(f.distance_remaining)
        self.recoveries = int(f.number_of_recoveries)

    def _onAccepted(self, fut, seq):
        h = fut.result()
        if seq != self._goalSeq or self._cancelPending:
            # Perime, ou annule AVANT d'etre accepte : on l'annule des maintenant.
            if h.accepted:
                h.cancel_goal_async()
            if seq == self._goalSeq:
                self._cancelPending = False
            return
        if not h.accepted:
            self.goal_status = "aborted"
            self.detail = "but %s refuse par Nav2" % self.goal_name
            return
        self._goal = h
        h.get_result_async().add_done_callback(lambda f, s=seq: self._onResult(f, s))

    def _onResult(self, fut, seq):
        if seq != self._goalSeq:
            return
        st = fut.result().status
        self.goal_status = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(st, str(st))
        self.detail = "but %s : %s" % (self.goal_name, self.goal_status)
        self._goal = None
        self.get_logger().info(self.detail)

    def _cancelGoal(self):
        if self._goal is not None:
            self._goal.cancel_goal_async()
            self._goal = None
            self.goal_status = "canceled"
        elif self.goal_status == "active":
            # Envoye mais pas encore accepte : _onAccepted l'annulera a son arrivee.
            self._cancelPending = True
            self.goal_status = "canceled"

    # ------------------------------------------------------------ commandes
    def _onCommand(self, req, resp):
        with self._lock:
            ok, msg = self.command(req.command.strip(), req.arg.strip())
        resp.success, resp.message = ok, msg
        self.detail = msg
        self._publish()
        return resp

    def command(self, cmd, arg):
        if cmd in ("exploration", "navigation", "idle"):
            return self.setSubmode(cmd, arg)
        if cmd == "auto_explore":
            if self.auto in ("running", "paused", "homing"):
                return self._autoStop("demande")
            return self._autoStart()
        if cmd == "save_map":
            cli = self.mapCli.get(self.submode)
            if cli is None:
                return False, "pas de carte en sous-mode %s (explorer d'abord)" % self.submode
            if not cli.wait_for_service(timeout_sec=2.0):
                return False, "%s absent" % cli.srv_name
            fut = cli.call_async(GetMap.Request())
            if not self._wait(fut, 10.0):
                return False, "%s sans reponse" % cli.srv_name
            m = fut.result().map
            name = arg or map_io.stamp_name("carte")
            q = m.info.origin.orientation
            path = map_io.write_map(os.path.join(self.maps_dir, name), m.info.width,
                                    m.info.height, m.info.resolution,
                                    (m.info.origin.position.x, m.info.origin.position.y,
                                     map_io.yaw_of(q.z, q.w, q.x, q.y)), m.data)
            known, occ = map_io.map_stats(m.info.width, m.info.height, m.data)
            if self.submode == "exploration":
                self.map_name = name
            return True, "carte %s (%dx%d, %.0f %% connue) -> %s" % (
                name, m.info.width, m.info.height, 100 * known, path)
        if cmd == "save_waypoint":
            if not self.map_name:
                return False, "aucune carte courante"
            pose = self._robotPose()
            if pose is None:
                return False, "pose inconnue (TF map->base_footprint)"
            wps = map_io.load_waypoints(self.maps_dir, self.map_name)
            name = arg or ("depart" if not wps else "p%d" % len(wps))
            map_io.save_waypoint(self.maps_dir, self.map_name, name, *pose)
            return True, "point %s enregistre (%.2f, %.2f)" % (name, pose[0], pose[1])
        if cmd == "goto":
            if self.submode != "navigation":
                return False, "passer d'abord en navigation"
            wps = map_io.load_waypoints(self.maps_dir, self.map_name)
            if not wps:
                return False, "aucun point sur la carte %s" % self.map_name
            if arg in ("", "next"):
                wp = wps[self._next % len(wps)]
                self._next += 1
            elif arg == "home":
                wp = next((w for w in wps if w["name"] == "depart"), wps[0])
            else:
                wp = next((w for w in wps if w["name"] == arg), None)
                if wp is None:
                    return False, "point %s inconnu" % arg
            return self._goto(wp)
        if cmd == "cancel":
            self._cancelGoal()
            return True, "but annule"
        if cmd == "list":
            maps = map_io.list_maps(self.maps_dir)
            wps = [w["name"] for w in map_io.load_waypoints(self.maps_dir, self.map_name)]
            return True, "cartes : %s ; points (%s) : %s" % (
                ", ".join(maps) or "aucune", self.map_name or "-", ", ".join(wps) or "aucun")
        return False, "commande inconnue : %s" % cmd

    # ------------------------------------------------------------ exploration automatique
    # Strategie (choisie le 2026-10-11) : aller de FRONTIERE en frontiere (limite connu /
    # inconnu, bamboo_navigation.frontier) jusqu'a ce qu'il n'en reste plus, puis REVENIR au
    # depart, sauver la carte et s'arreter. Le stick met en PAUSE (le multiplexeur lui donne
    # deja la main) ; l'exploration repart seule `auto_resume_s` apres le relachement.
    def _autoStart(self):
        if self.submode != "exploration":
            ok, why = self.setSubmode("exploration")
            if not ok:
                return False, "exploration auto impossible : " + why
        ok, why = self._manage(self.navMgr, STARTUP)
        if not ok:
            return False, "exploration auto impossible : Nav2 " + why
        self.auto, self._autoResumeTo = "running", "running"
        self._autoT0 = time.monotonic()
        self._autoHome, self._autoTarget = None, None
        self._autoAvoid, self._autoN, self._autoNoFront, self._autoHomeTries = [], 0, 0, 0
        self.get_logger().info("exploration auto : demarrage")
        return True, "exploration auto lancee"

    def _autoStop(self, why, nav_reset=True):
        self._autoOwnCancel = True
        self._cancelGoal()
        if nav_reset and self.submode == "exploration":
            # Exploration MANUELLE ensuite : Nav2 n'y sert a rien (cf. setSubmode).
            self._manage(self.navMgr, RESET)
        self.auto, self._autoTarget = "off", None
        self.detail = "exploration auto arretee (%s)" % why
        self.get_logger().info(self.detail)
        return True, self.detail

    def _autoGoto(self, name, x, y, yaw):
        self._autoTarget, self._autoGoalT, self._autoOwnCancel = (x, y), time.monotonic(), False
        return self._goto({"name": name, "x": x, "y": y, "yaw": yaw})

    def _autoMap(self):
        """Carte EN COURS de slam_toolbox, demandee hors du verrou (appel de service)."""
        cli = self.mapCli["exploration"]
        if not cli.service_is_ready():
            return None
        fut = cli.call_async(GetMap.Request())
        if not self._wait(fut, 5.0):
            return None
        return fut.result().map

    def _autoTick(self):
        if self.auto in ("off", "done"):
            return
        now = time.monotonic()
        # --- garde-fous, du plus grave au moins grave ---
        if self._robotMode and self._robotMode != "navigation":
            with self._lock:
                self._autoStop("sortie du mode NAV")
            return
        if "lidar" in self.required and not self._fresh("scan"):
            with self._lock:
                self._autoStop("lidar muet")
            return
        if self.auto in ("running", "paused"):
            low = not math.isnan(self._battV) and 0.0 < self._battV < self.auto_battery_min_v
            late = now - self._autoT0 > self.auto_max_duration_s
            if low or late:
                self._autoHoming("batterie %.2f V" % self._battV if low else "duree maximale")
        # --- pause manette : le stick a la main, on n'envoie RIEN ---
        if now - self._teleopT < self.auto_resume_s:
            if self.auto in ("running", "homing"):
                with self._lock:
                    self._autoResumeTo, self.auto = self.auto, "paused"
                    self._autoOwnCancel = True
                    self._cancelGoal()
                    self._autoTarget = None
                    self.detail = "exploration auto : PAUSE (manette)"
            return
        if self.auto == "paused":
            self.auto = self._autoResumeTo
        pose = self._robotPose()
        if pose is None:
            self.detail = "exploration auto : attente de la pose (map->base_footprint)"
            return
        if self._autoHome is None:
            self._autoHome = pose
            self.get_logger().info("exploration auto : depart (%.2f, %.2f)" % pose[:2])
        # --- but en cours ---
        if self.goal_status == "active":
            if now - self._autoGoalT > self.auto_goal_timeout_s:
                with self._lock:
                    self.get_logger().warn("exploration auto : %s trop long, abandonne"
                                           % self.goal_name)
                    self._autoOwnCancel = False
                    self._cancelGoal()
            else:
                return
        # --- bilan du but termine ---
        if self._autoTarget is not None:
            ok = self.goal_status == "succeeded"
            if self.auto == "homing":
                if ok:
                    with self._lock:
                        self._autoFinish()
                    return
                self._autoHomeTries += 1
                if self._autoHomeTries >= 3:
                    with self._lock:
                        self._autoStop("retour au depart impossible")
                    return
            elif not self._autoOwnCancel:
                # Atteint OU echoue : dans les deux cas on ne le revise pas (un but atteint
                # reste parfois en bord d'inconnu, et le robot y reviendrait en boucle).
                self._autoAvoid.append(self._autoTarget)
                if not ok:
                    self.get_logger().info("exploration auto : %s %s, liste noire"
                                           % (self.goal_name, self.goal_status))
            self._autoTarget = None
        if self.auto == "homing":
            self._autoGotoHome()
            return
        # --- frontiere suivante ---
        m = self._autoMap()
        if m is None:
            self.detail = "exploration auto : carte indisponible (slam_toolbox ?)"
            return
        grid = frontier.as_grid(m.data, m.info.width, m.info.height)
        fs = frontier.find_frontiers(grid, m.info.resolution,
                                     (m.info.origin.position.x, m.info.origin.position.y),
                                     min_size_m=self.auto_min_frontier_m)
        self._autoKnown, _occ = map_io.map_stats(m.info.width, m.info.height, m.data)
        f = frontier.choose(fs, pose[:2], self._autoAvoid)
        self._autoFronts = len(fs)
        if f is None:
            self._autoNoFront += 1
            self.detail = "exploration auto : plus de frontiere (%d/2)" % self._autoNoFront
            if self._autoNoFront >= 2:
                self._autoHoming("carte complete")
            return
        self._autoNoFront = 0
        self._autoN += 1
        yaw = math.atan2(f["y"] - pose[1], f["x"] - pose[0])
        with self._lock:
            self._autoGoto("frontiere %d" % self._autoN, f["x"], f["y"], yaw)
        self.detail = "exploration auto : frontiere %d, %d restantes, %.0f %% connue" % (
            self._autoN, len(fs), 100 * self._autoKnown)
        self.get_logger().info("exploration auto : frontiere %d -> (%.2f, %.2f), %.2f m de "
                               "large, %d en vue" % (self._autoN, f["x"], f["y"],
                                                     f["size_m"], len(fs)))

    def _autoHoming(self, why):
        if self.auto == "homing":
            return
        self.get_logger().info("exploration auto : retour au depart (%s)" % why)
        with self._lock:
            self._autoOwnCancel = True
            self._cancelGoal()
            self.auto, self._autoResumeTo, self._autoTarget = "homing", "homing", None
            self.detail = "exploration auto : retour au depart (%s)" % why

    def _autoGotoHome(self):
        if self._autoHome is None:
            with self._lock:
                self._autoStop("depart inconnu")
            return
        x, y, yaw = self._autoHome
        with self._lock:
            self._autoGoto("retour depart", x, y, yaw)

    def _autoFinish(self):
        ok, msg = self.command("save_map", "")
        if ok and self._autoHome is not None:
            map_io.save_waypoint(self.maps_dir, self.map_name, "depart", *self._autoHome)
        self._manage(self.navMgr, RESET)
        self.auto, self._autoTarget = "done", None
        self.detail = "exploration auto TERMINEE en %.0f s : %s" % (
            time.monotonic() - self._autoT0, msg)
        self.get_logger().info(self.detail)

    # ------------------------------------------------------------ etat et diagnostic
    def _publish(self):
        m = NavState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.submode, m.map_name = self.submode, self.map_name
        m.goal, m.goal_status = self.goal_name, self.goal_status
        m.distance_remaining = float(self.distance)
        m.recoveries = int(self.recoveries)
        m.localization_cov = float(self.loc_cov) if self.submode == "navigation" else math.nan
        m.waypoints = [w["name"] for w in map_io.load_waypoints(self.maps_dir, self.map_name)] \
            if self.map_name else []
        m.missing_sensors = self.missing()
        m.ready = not m.missing_sensors
        m.detail = self.detail
        self.pubState.publish(m)

    def _diagNav(self, stat):
        miss = self.missing()
        stat.add("sous_mode", self.submode)
        stat.add("carte", self.map_name or "-")
        stat.add("capteurs_exiges", ", ".join(self.required) or "aucun")
        stat.add("manquants", ", ".join(miss) or "aucun")
        stat.add("but", "%s %s" % (self.goal_name or "-", self.goal_status))
        stat.add("auto", self.auto)
        if self.auto != "off":
            stat.add("frontieres", str(self._autoFronts))
            stat.add("connue_pct", "%.0f" % (100 * self._autoKnown)
                     if not math.isnan(self._autoKnown) else "-")
            stat.add("restant_s", "%.0f" % max(0.0, self.auto_max_duration_s -
                                               (time.monotonic() - self._autoT0)))
        lidar_missing = [x for x in miss if x.startswith("lidar")]
        if not self.required or "lidar" not in self.required:
            stat.summary(DiagnosticStatus.ERROR, "navigation impossible : ce robot n'a pas "
                                                 "de lidar (sensors.lidar)")
        elif lidar_missing:
            stat.summary(DiagnosticStatus.ERROR, "navigation impossible : " + lidar_missing[0])
        elif miss:
            stat.summary(DiagnosticStatus.WARN, "manquant : " + ", ".join(miss))
        elif self.submode == "navigation" and not math.isnan(self.loc_cov) and \
                self.loc_cov > self.loc_cov_warn:
            stat.summary(DiagnosticStatus.WARN, "localisation incertaine (cov %.2f)"
                         % self.loc_cov)
        else:
            stat.summary(DiagnosticStatus.OK, "pret (%s%s)" % (
                self.submode, (", " + self.map_name) if self.map_name else ""))
        return stat

    def shutdown(self):
        self._cancelGoal()
        self._stopSlam()


def main(argv=None):
    rclpy.init(args=argv)
    node = NavSupervisor()
    # 3 fils : l'exploration auto attend des services (carte, Nav2) dans sa minuterie, et une
    # commande de la manette doit pouvoir passer pendant ce temps.
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node._watchEx.shutdown()
        node._watch.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
