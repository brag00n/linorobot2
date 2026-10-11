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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformListener

from bamboo_interfaces.msg import NavState
from bamboo_interfaces.srv import NavCommand
from bamboo_navigation import map_io

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

        self.submode = "idle"
        self.map_name = ""
        self.slam = None              # sous-processus slam_toolbox
        self.goal_name, self.goal_status = "", ""
        self.distance, self.recoveries = math.nan, 0
        self.loc_cov = math.nan
        self.detail = "pret"
        self._goal = None
        self._next = 0
        self._lock = threading.Lock()
        self._last = {"scan": None, "imu": None, "odom": None}

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
        os.makedirs(self.maps_dir, exist_ok=True)
        self.get_logger().info("superviseur de navigation : cartes dans %s, capteurs exiges %s"
                               % (self.maps_dir, self.required or "aucun"))

    # ------------------------------------------------------------ capteurs
    def _seen(self, k):
        self._last[k] = time.monotonic()

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
        fut = self.nav.send_goal_async(goal, feedback_callback=self._onFeedback)
        fut.add_done_callback(self._onAccepted)
        return True, "en route vers %s" % wp["name"]

    def _onFeedback(self, fb):
        f = fb.feedback
        self.distance = float(f.distance_remaining)
        self.recoveries = int(f.number_of_recoveries)

    def _onAccepted(self, fut):
        h = fut.result()
        if not h.accepted:
            self.goal_status = "aborted"
            self.detail = "but %s refuse par Nav2" % self.goal_name
            return
        self._goal = h
        h.get_result_async().add_done_callback(self._onResult)

    def _onResult(self, fut):
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
    # 2 fils : un pour les commandes (qui attendent des services), un pour le reste. Chaque
    # fil de plus se reveille aussi a chaque message, pour rien.
    ex = MultiThreadedExecutor(num_threads=2)
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
