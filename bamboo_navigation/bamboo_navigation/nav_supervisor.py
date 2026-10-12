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
import collections
import json
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
from geometry_msgs.msg import (Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped,
                               Twist)
from std_msgs.msg import Bool, String
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, ManageLifecycleNodes
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from tf2_ros import Buffer, TransformListener

from bamboo_interfaces.msg import NavState, RobotMode, TeleopState
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
        # Delai SANS PROGRES (la distance restante ne baisse plus) avant d'abandonner un but ;
        # un but qui progresse est garde jusqu'a 3 fois ce delai. Le retour au depart a le sien :
        # essai E2b, un retour de l'autre bout de la piece depassait 60 s et etait abandonne
        # 2 s avant d'arriver.
        self.auto_goal_timeout_s = float(d("auto_goal_timeout_s", 60.0).value)
        self.auto_home_timeout_s = float(d("auto_home_timeout_s", 180.0).value)
        self.auto_max_duration_s = float(d("auto_max_duration_s", 900.0).value)
        self.auto_resume_s = float(d("auto_resume_s", 3.0).value)
        # Batterie jugee sur le POURCENTAGE de /battery, que le driver calcule sur la tension AU
        # REPOS (traction arretee). La tension instantanee chute de ~1,4 V sous l'effort des
        # moteurs : meme en mediane sur 20 s, elle a renvoye le robot au depart avec une
        # batterie a 72 % (essai E2d, 2026-10-12). Repli sur la tension (mediane, seuil bas)
        # seulement si le pourcentage est inconnu.
        self.auto_battery_min_pct = float(d("auto_battery_min_pct", 15.0).value)
        self.auto_battery_min_v = float(d("auto_battery_min_v", 10.0).value)
        self.auto_battery_window_s = float(d("auto_battery_window_s", 20.0).value)
        # Recalibration du gyro AVANT de partir (robot immobile) : service `recalibrate_imu`
        # du driver, trouve automatiquement si vide (ESP32 comme STM32).
        self.auto_recal_service = str(d("auto_recal_service", "").value)
        self.auto_recal_settle_s = float(d("auto_recal_settle_s", 4.0).value)
        self.auto_still_s = float(d("auto_still_s", 2.0).value)
        teleop_topic = str(d("teleop_topic", "/cmd_vel_teleop").value)
        battery_topic = str(d("battery_topic", "/battery").value)
        mode_topic = str(d("mode_topic", "/bamboo/mode").value)
        teleop_state_topic = str(d("teleop_state_topic", "/teleop/state").value)

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
        self._autoFailedPts = []         # echecs de 1re passe : retentes UNE fois a la fin
        self._autoSecond = False         # seconde chance deja donnee ?
        self._autoProgT = 0.0            # dernier progres du but (distance restante en baisse)
        self._autoBestD = math.inf
        # DEBRIEFING (demande utilisateur 2026-10-11) : chaque frontiere, numerotee, avec son
        # statut ; publie en continu, enregistre avec la carte, dessine par l'outil MCP `map`.
        self._autoLog = []
        self._autoEntry = None
        self._autoN = 0
        self._autoNoFront = 0
        self._autoHomeTries = 0
        self._autoFronts = 0
        self._autoKnown = math.nan
        self._teleopT = 0.0              # dernier ordre NON NUL de la manette
        self._battV = math.nan           # mediane glissante (cf. _onBattery)
        self._battPct = math.nan         # etat de charge AU REPOS (0..100), du driver
        self._lastSaveT = -1e9           # derniere sauvegarde de carte (dedoublonnage)
        self._autoClearT = -1e9          # dernier vidage des costmaps (echecs en serie)
        self._battHist = collections.deque()
        self._robotMode = ""
        self._lastRobotMode = ""
        self._armed = None               # traction armee a la manette ? None = inconnu (simu)

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
        self._watch.create_subscription(BatteryState, battery_topic, self._onBattery, 1)
        latched_in = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._watch.create_subscription(
            RobotMode, mode_topic, lambda m: setattr(self, "_robotMode", m.mode), latched_in)
        # Traction DESARMEE = pause : les roues ne tournent plus, chaque but finirait en echec
        # et TOUTES les frontieres passeraient en liste noire (essai E1, 2026-10-11).
        self._watch.create_subscription(
            TeleopState, teleop_state_topic, lambda m: setattr(self, "_armed", bool(m.armed)),
            latched_in)
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
        # AUTORISATION de la voie navigation du multiplexeur : ouverte SEULEMENT pendant un
        # but actif. A l'arret, Nav2 met ~1 s a s'eteindre et ses manoeuvres de reprise
        # publient en direct : sans ce verrou, soubresauts (essai E1, 2026-10-11).
        self.pubEnable = self.create_publisher(
            Bool, str(d("enable_topic", "/navigation/enable").value), latched)
        self._navEnabled = None
        self._setNavEnable(False)
        # Pour la MINI-CARTE du HUD (overlay_node) : pose du robot dans map (4 Hz, hors idle :
        # c'est nous qui ecoutons deja la TF), depart et frontieres de l'exploration auto.
        self.pubPose = self.create_publisher(PoseStamped, "/navigation/robot_pose", 1)
        self.pubHome = self.create_publisher(PoseStamped, "/navigation/home", latched)
        self.pubFronts = self.create_publisher(PoseArray, "/navigation/frontiers", latched)
        self.pubLog = self.create_publisher(String, "/navigation/frontier_log", latched)
        self.create_timer(0.25, self._publishPose, callback_group=self._cbg)

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
        # Pour verifier qu'une frontiere est ATTEIGNABLE avant d'y envoyer le robot.
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose",
                                    callback_group=self._cbg)
        self.clearCli = [self.create_client(ClearEntireCostmap, s, callback_group=self._cbg)
                         for s in ("/global_costmap/clear_entirely_global_costmap",
                                   "/local_costmap/clear_entirely_local_costmap")]
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

    def _onBattery(self, m):
        pct = float(m.percentage)
        self._battPct = 100.0 * pct if pct == pct and 0.0 <= pct <= 1.0 else math.nan
        now, v = time.monotonic(), float(m.voltage)
        if not v > 0.0:
            return
        h = self._battHist
        h.append((now, v))
        while h and now - h[0][0] > self.auto_battery_window_s:
            h.popleft()
        vals = sorted(x for _, x in h)
        self._battV = vals[len(vals) // 2]

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
            # Sur le SATELLITE (executeur mono-fil) : sous le MultiThreadedExecutor du
            # superviseur, les ~40 messages/s de /tf portaient le noeud a 67 % d'un coeur en
            # exploration (essai E1b, 2026-10-11). Le Buffer est sur entre fils.
            self.tfl = TransformListener(self.tf, self._watch)
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
    def _slamAlive(self):
        return self.slam is not None and self.slam.poll() is None

    def _saveBeforeLeaving(self, why):
        """Sauve la carte EN COURS avant que slam_toolbox ne s'arrete. Essai E2c (2026-10-11) :
        Y long (idle) puis X long -> carte PERDUE, slam_toolbox etait deja arrete."""
        if self.submode != "exploration" or not self._slamAlive():
            return ""
        if time.monotonic() - self._lastSaveT < 10.0:
            return ""                          # deja sauvee a l'instant (deux causes d'arret)
        ok, msg = self.command("save_map", "")
        if ok:
            self._lastSaveT = time.monotonic()
        self.get_logger().info("carte sauvee avant %s : %s" % (why, msg) if ok else
                               "carte NON sauvee avant %s : %s" % (why, msg))
        return msg if ok else ""

    def setSubmode(self, mode, arg=""):
        if mode == self.submode and mode != "navigation":
            return True, "deja en %s" % mode
        saved = ""
        if mode != "exploration":
            saved = self._saveBeforeLeaving("le sous-mode %s" % mode)
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
        if saved:
            self.detail += " ; " + saved
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

    def _publishPose(self):
        if self.tfl is None:
            return
        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time())
        except Exception:                                   # noqa: BLE001
            return
        m = PoseStamped()
        m.header.frame_id, m.header.stamp = "map", t.header.stamp
        m.pose.position.x = t.transform.translation.x
        m.pose.position.y = t.transform.translation.y
        m.pose.orientation = t.transform.rotation
        self.pubPose.publish(m)

    def _publishMarks(self, fronts=()):
        """Depart et frontieres pour la mini-carte ; rien = efface (frame_id vide)."""
        h = PoseStamped()
        if self.auto != "off" and self._autoHome is not None:
            h.header.frame_id = "map"
            h.pose.position.x, h.pose.position.y = self._autoHome[0], self._autoHome[1]
        self.pubHome.publish(h)
        a = PoseArray()
        a.header.frame_id = "map"
        for f in fronts:
            p = Pose()
            p.position.x, p.position.y = f["x"], f["y"]
            a.poses.append(p)
        self.pubFronts.publish(a)

    def _setNavEnable(self, on):
        if on != self._navEnabled:
            self._navEnabled = on
            self.pubEnable.publish(Bool(data=on))

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
        self._setNavEnable(True)
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
            self._setNavEnable(False)
            return
        self._goal = h
        h.get_result_async().add_done_callback(lambda f, s=seq: self._onResult(f, s))

    def _onResult(self, fut, seq):
        if seq != self._goalSeq:
            return
        self._setNavEnable(False)
        st = fut.result().status
        self.goal_status = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(st, str(st))
        self.detail = "but %s : %s" % (self.goal_name, self.goal_status)
        self._goal = None
        self.get_logger().info(self.detail)

    def _cancelGoal(self):
        self._setNavEnable(False)          # d'abord : la voie se ferme dans la milliseconde
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
            if self._autoLog:
                # Le DEBRIEFING de l'exploration auto suit TOUTE sauvegarde de la carte (X long
                # compris) : <carte>_frontieres.yaml.
                map_io.save_frontier_log(self.maps_dir, name, self._autoLog, self._autoHome)
                self._publishLog()
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
    def _recalService(self):
        """Service `recalibrate_imu` du driver : le parametre, sinon le premier du graphe."""
        if self.auto_recal_service:
            return self.auto_recal_service
        for name, types in self.get_service_names_and_types():
            if name.endswith("/recalibrate_imu") and "std_srvs/srv/Trigger" in types:
                return name
        return ""

    def _recalGyro(self):
        """Fait remesurer le biais du gyro par la carte (robot IMMOBILE). Retourne
        (instant de la demande, message) ; instant None si rien n'a ete fait.

        Essai Nc (2026-10-11) : le gyro derive de 0,12 deg/min a l'arret, et l'EKF l'integre
        comme une rotation ; 6,6 des 9,2 deg de cap que le SLAM a du corriger, et des murs
        dedoubles. Une remesure juste avant de partir retire le biais pris depuis le boot
        (temperature). Elle ne suffit pas aux longues immobilites : cf. le gyro ignore a
        l'arret."""
        name = self._recalService()
        if not name:
            return None, "gyro NON recalibre (aucun service recalibrate_imu)"
        cli = self.create_client(Trigger, name, callback_group=self._cbg)
        try:
            if not cli.wait_for_service(timeout_sec=1.0):
                return None, "gyro NON recalibre (%s absent)" % name
            fut = cli.call_async(Trigger.Request())
            if not self._wait(fut, 3.0) or not fut.result().success:
                return None, "gyro NON recalibre (%s : %s)" % (
                    name, fut.result().message if fut.done() and fut.result() else "muet")
            return time.monotonic(), "gyro recalibre (%s)" % name
        finally:
            self.destroy_client(cli)

    def _autoStart(self):
        # Le robot doit etre IMMOBILE : la carte moyenne le gyro ~2 s, et un mouvement
        # pendant ce temps deviendrait un biais.
        if time.monotonic() - self._teleopT < self.auto_still_s:
            return False, ("exploration auto refusee : robot en mouvement (lacher le stick "
                           "%.0f s)" % self.auto_still_s)
        # Carte NEUVE a chaque lancement, partie de la position REELLE du robot. Reprendre la
        # session de slam_toolbox en cours etait dangereux : un robot deplace a la main entre
        # temps (les roues n'ont rien vu, le lidar si) part sur une carte et une pose fausses
        # (essai E2 du 2026-10-11 : « il part sur une ancienne carte »).
        if self.submode == "exploration":
            # La carte en cours est SAUVEE avant d'etre remplacee par une neuve.
            self._saveBeforeLeaving("la relance de l'exploration")
            self._manage(self.navMgr, RESET)
            self._stopSlam()
            self.tf.clear()            # sinon l'ANCIEN map->odom reste quelques secondes
            ok, why = self._startSlam()
            if not ok:
                return False, "exploration auto impossible : " + why
            self.map_name = map_io.stamp_name("exploration")
        else:
            ok, why = self.setSubmode("exploration")
            if not ok:
                return False, "exploration auto impossible : " + why
        recal_t, recal_msg = self._recalGyro()
        if recal_t is None:
            self.get_logger().warn("exploration auto : " + recal_msg)
        # Le demarrage de Nav2 (~10 s) recouvre l'attente de la recalibration.
        ok, why = self._manage(self.navMgr, STARTUP)
        if not ok:
            return False, "exploration auto impossible : Nav2 " + why
        if recal_t is not None:
            rest = self.auto_recal_settle_s - (time.monotonic() - recal_t)
            if rest > 0:
                time.sleep(rest)
        self.auto, self._autoResumeTo = "running", "running"
        self._autoT0 = time.monotonic()
        self._autoHome, self._autoTarget = None, None
        self._autoAvoid, self._autoN, self._autoNoFront, self._autoHomeTries = [], 0, 0, 0
        self._autoFailedPts, self._autoSecond = [], False
        self._autoLog, self._autoEntry = [], None
        self._publishLog()
        self.get_logger().info("exploration auto : demarrage, " + recal_msg)
        return True, "exploration auto lancee, " + recal_msg

    def _autoStop(self, why, nav_reset=True):
        # L'etat bascule EN PREMIER : la boucle peut etre au milieu d'un tour (carte demandee
        # hors verrou) ; elle revoit `auto` sous le verrou avant tout envoi. Essai E1 : sans
        # cela, une frontiere est partie 2 s APRES l'arret.
        self.auto, self._autoTarget = "off", None
        # TOUT arret sauve la carte (essai E2d : arret sur "retour au depart impossible", puis
        # relance -> carte de la 1re exploration PERDUE).
        self._saveBeforeLeaving("l'arret de l'exploration auto (%s)" % why)
        self._publishMarks()
        self._autoOwnCancel = True
        self._cancelGoal()
        if nav_reset and self.submode == "exploration":
            # Exploration MANUELLE ensuite : Nav2 n'y sert a rien (cf. setSubmode).
            self._manage(self.navMgr, RESET)
        self.detail = "exploration auto arretee (%s)" % why
        self.get_logger().info(self.detail)
        return True, self.detail

    def _autoGoto(self, name, x, y, yaw):
        self._autoTarget, self._autoGoalT, self._autoOwnCancel = (x, y), time.monotonic(), False
        self._autoProgT, self._autoBestD = self._autoGoalT, math.inf
        return self._goto({"name": name, "x": x, "y": y, "yaw": yaw})

    # --- debriefing -------------------------------------------------------------------
    # Statuts : en_cours, atteinte, abandonnee (Nav2), trop_longue (sans progres),
    # inatteignable (aucun chemin, ecartee sans y aller), interrompue (pause manette).
    def _logEntry(self, x, y, status):
        self._autoN += 1
        e = {"n": self._autoN, "x": round(float(x), 2), "y": round(float(y), 2), "s": status}
        self._autoLog.append(e)
        self._publishLog()
        return e

    def _logSet(self, e, status):
        if e is not None and e["s"] != status:
            e["s"] = status
            self._publishLog()

    def _publishLog(self):
        doc = {"map": self.map_name, "home": list(self._autoHome[:2]) if self._autoHome else None,
               "frontiers": self._autoLog}
        self.pubLog.publish(String(data=json.dumps(doc)))

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
        if self.auto in ("off", "done", "finishing"):
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
            if not math.isnan(self._battPct):
                low = self._battPct < self.auto_battery_min_pct
                batt = "batterie %.0f %% (au repos)" % self._battPct
            else:
                low = not math.isnan(self._battV) and 0.0 < self._battV < self.auto_battery_min_v
                batt = "batterie %.2f V" % self._battV
            late = now - self._autoT0 > self.auto_max_duration_s
            if low or late:
                self._autoHoming(batt if low else "duree maximale")
        # --- pause : le stick a la main, ou la traction est desarmee ; on n'envoie RIEN ---
        stick = now - self._teleopT < self.auto_resume_s
        if stick or self._armed is False:
            if self.auto in ("running", "homing"):
                with self._lock:
                    self._autoResumeTo, self.auto = self.auto, "paused"
                    self._autoOwnCancel = True
                    self._cancelGoal()
                    self._autoTarget = None
                    self._logSet(self._autoEntry, "interrompue")
                    self._autoEntry = None
            self.detail = "exploration auto : PAUSE (%s)" % (
                "manette" if stick else "traction desarmee : B long pour reprendre")
            return
        if self.auto == "paused":
            with self._lock:
                if self.auto != "paused":
                    return
                self.auto = self._autoResumeTo
        pose = self._robotPose()
        if pose is None:
            self.detail = "exploration auto : attente de la pose (map->base_footprint)"
            return
        if self._autoHome is None:
            self._autoHome = pose
            self._publishMarks()
            self.get_logger().info("exploration auto : depart (%.2f, %.2f)" % pose[:2])
        # --- but en cours ---
        if self.goal_status == "active":
            # Abandon sur ABSENCE DE PROGRES (distance restante qui ne baisse plus), plafonne a
            # 3 fois le delai : un but qui avance n'est pas coupe.
            if not math.isnan(self.distance) and self.distance < self._autoBestD - 0.10:
                self._autoBestD, self._autoProgT = self.distance, now
            limit = self.auto_home_timeout_s if self.auto == "homing" \
                else self.auto_goal_timeout_s
            if now - self._autoProgT > limit or now - self._autoGoalT > 3 * limit:
                with self._lock:
                    self.get_logger().warn("exploration auto : %s sans progres, abandonne"
                                           % self.goal_name)
                    self._autoOwnCancel = False
                    self._cancelGoal()
                    self._logSet(self._autoEntry, "trop_longue")
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
                if time.monotonic() - self._autoClearT > 30.0:
                    # Meme cause probable qu'en exploration : costmap qui croit le robot dans
                    # un obstacle. On vide avant de recompter.
                    self._clearCostmaps("retour au depart en echec")
                if self._autoHomeTries >= 3:
                    with self._lock:
                        self._autoStop("retour au depart impossible")
                    return
            elif not self._autoOwnCancel:
                # Atteint OU echoue : dans les deux cas on ne le revise pas (un but atteint
                # reste parfois en bord d'inconnu, et le robot y reviendrait en boucle). Un
                # ECHEC est retente une fois a la fin (seconde chance, cf. plus bas).
                self._autoAvoid.append(self._autoTarget)
                if ok:
                    self._logSet(self._autoEntry, "atteinte")
                else:
                    self._autoFailedPts.append(self._autoTarget)
                    if self._autoEntry is not None and self._autoEntry["s"] == "en_cours":
                        self._logSet(self._autoEntry, "abandonnee")
                    self.get_logger().info("exploration auto : %s %s, liste noire"
                                           % (self.goal_name, self.goal_status))
            self._autoTarget = None
            self._autoEntry = None
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
        self._autoFronts = len(fs)
        self._publishMarks(fs)
        # On ne vise qu'une frontiere ATTEIGNABLE : demander d'abord un chemin au planificateur
        # (< 1 s). Essai E2 (2026-10-11) : 4 frontieres sur 6 etaient sous des meubles (le
        # lidar, a 17 cm, voit sous un canape ; le robot n'y passe pas) ; Nav2 y perdait ~30 s
        # chacune en manoeuvres de reprise avant d'abandonner.
        f, bad = None, []
        for _try in range(4):
            cand = frontier.choose(fs, pose[:2], self._autoAvoid + bad)
            if cand is None:
                break
            ok_plan = self._planOk(cand["x"], cand["y"])
            if ok_plan is not False:           # chemin trouve, ou verification impossible
                f = cand
                break
            bad.append((cand["x"], cand["y"]))
            if self.auto != "running":
                return
        # ECHECS EN SERIE : 3 frontieres d'affilee sans chemin, c'est presque toujours le ROBOT
        # qui est dans une zone que la costmap croit occupee (arrete au ras d'un obstacle), pas
        # trois frontieres inaccessibles -- essai E2d : 11 condamnees en 2 s. On vide les
        # costmaps et on reessaie au tour suivant AVANT de condamner quoi que ce soit.
        if len(bad) >= 3 and time.monotonic() - self._autoClearT > 60.0:
            self._clearCostmaps("%d frontieres sans chemin d'affilee" % len(bad))
            return
        for bx, by in bad:
            self._autoAvoid.append((bx, by))
            e = self._logEntry(bx, by, "inatteignable")
            self.get_logger().info("exploration auto : frontiere %d (%.2f, %.2f) INATTEIGNABLE, "
                                   "ecartee" % (e["n"], bx, by))
        if f is None:
            if frontier.choose(fs, pose[:2], self._autoAvoid) is not None:
                return                         # d'autres candidates : au tour suivant
            if not self._autoSecond and self._autoFailedPts:
                # SECONDE CHANCE, une fois : une frontiere abandonnee (robot coince contre un
                # chambranle, essai E2b : la porte du haut) redevient souvent atteignable avec
                # la carte agrandie et depuis une autre position. Les INATTEIGNABLES (aucun
                # chemin) ne sont pas retentees.
                self._autoSecond = True
                failed = self._autoFailedPts
                self._autoAvoid = [p for p in self._autoAvoid if p not in failed]
                self._autoFailedPts = []
                self.get_logger().info("exploration auto : seconde chance pour %d frontieres "
                                       "abandonnees" % len(failed))
                return
            self._autoNoFront += 1
            reason = "carte complete" if not fs else \
                "plus de frontiere atteignable (%d ecartees)" % len(fs)
            self.detail = "exploration auto : %s (%d/2)" % (reason, self._autoNoFront)
            if self._autoNoFront >= 2:
                self._autoHoming(reason)
            return
        self._autoNoFront = 0
        yaw = math.atan2(f["y"] - pose[1], f["x"] - pose[0])
        with self._lock:
            if self.auto != "running":     # arrete ou mis en pause pendant ce tour
                return
            ok, why = self._autoGoto("frontiere %d" % (self._autoN + 1), f["x"], f["y"], yaw)
            if not ok:                     # Nav2 absent : rien n'est parti, on le dit
                self._autoTarget = None
                self.detail = "exploration auto : " + why
                return
            self._autoEntry = self._logEntry(f["x"], f["y"], "en_cours")
            self.detail = "exploration auto : frontiere %d, %d restantes, %.0f %% connue" % (
                self._autoN, len(fs), 100 * self._autoKnown)
        self.get_logger().info("exploration auto : frontiere %d -> (%.2f, %.2f), %.2f m de "
                               "large, %d en vue" % (self._autoN, f["x"], f["y"],
                                                     f["size_m"], len(fs)))

    def _planOk(self, x, y):
        """Le planificateur trouve-t-il un chemin jusqu'a (x, y) ? None si on ne sait pas."""
        if not self.planner.wait_for_server(timeout_sec=1.0):
            return None
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = "map"
        g.goal.header.stamp = self.get_clock().now().to_msg()
        g.goal.pose.position.x, g.goal.pose.position.y = float(x), float(y)
        g.goal.pose.orientation.w = 1.0
        g.use_start = False
        fut = self.planner.send_goal_async(g)
        if not self._wait(fut, 3.0):
            return None
        h = fut.result()
        if not h.accepted:
            return None
        res = h.get_result_async()
        if not self._wait(res, 5.0):
            return None
        r = res.result()
        return r.status == 4 and len(r.result.path.poses) > 0      # 4 = SUCCEEDED

    def _clearCostmaps(self, why):
        """Vide les deux costmaps (le lidar les remplit de nouveau en quelques scans)."""
        self._autoClearT = time.monotonic()
        n = 0
        for cli in self.clearCli:
            if cli.service_is_ready():
                cli.call_async(ClearEntireCostmap.Request())
                n += 1
        self.get_logger().warn("exploration auto : %s -> costmaps videes (%d), nouvel essai"
                               % (why, n))
        self.detail = "exploration auto : costmaps videes (%s)" % why

    def _autoHoming(self, why):
        if self.auto == "homing":
            return
        self.get_logger().info("exploration auto : retour au depart (%s)" % why)
        with self._lock:
            # 4. Debriefing : la frontiere coupee par le retour est INTERROMPUE, pas en_cours.
            self._logSet(self._autoEntry, "interrompue")
            self._autoEntry = None
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
            if self.auto != "homing":
                return
            ok, why = self._autoGoto("retour depart", x, y, yaw)
            if not ok:
                self._autoTarget = None
                self.detail = "exploration auto : retour impossible, " + why

    def _autoFinish(self):
        # Une seule fois : l'arret de Nav2 qui suit dure ~30 s, et la boucle, qui avait vu le
        # retour reussi, rappelait cette fin a chaque tour (carte sauvee 3 fois, essai E2).
        if self.auto != "homing":
            return
        self.auto = "finishing"
        ok, msg = self.command("save_map", "")
        if ok and self._autoHome is not None:
            map_io.save_waypoint(self.maps_dir, self.map_name, "depart", *self._autoHome)
        self._manage(self.navMgr, RESET)
        self.auto, self._autoTarget = "done", None
        self._publishMarks()
        self.detail = "exploration auto TERMINEE en %.0f s : %s" % (
            time.monotonic() - self._autoT0, msg)
        self.get_logger().info(self.detail)

    # ------------------------------------------------------------ etat et diagnostic
    def _watchModeExit(self):
        """Sortie du mode NAV : avec la navigation A LA DEMANDE, ce conteneur sera arrete 2 min
        plus tard, et slam_toolbox avec lui -- on sauve donc la carte DES la sortie, une fois."""
        mode = self._robotMode
        if mode and mode != "navigation" and self._lastRobotMode == "navigation":
            with self._lock:
                self._saveBeforeLeaving("la sortie du mode NAV")
        if mode:
            self._lastRobotMode = mode

    def _publish(self):
        self._watchModeExit()
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
