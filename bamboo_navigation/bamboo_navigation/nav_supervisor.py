#!/usr/bin/env python3
"""nav_supervisor -- les SOUS-MODES de la navigation, la carte COURANTE, les gates, les
destinations, la relocalisation et le diagnostic.

LA CARTE COURANTE (objectifs utilisateur du 2026-10-12) : une carte est un dossier
maps/<nom>/ (map_io.MapStore) qui porte le GRAPHE DE POSES de slam_toolbox, l'image, les
destinations, les gates et l'historique. Elle est gardee d'un sous-mode a l'autre :
  exploration  slam_toolbox en CARTOGRAPHIE sur la carte courante : elle se COMPLETE (graphe
               recharge a la pose du robot), ou repart vierge (`new_map`). Exploration
               automatique de GATE en gate (zones inconnues numerotees), ou a la manette.
  navigation   slam_toolbox en LOCALISATION sur le graphe de la carte courante, Nav2 actif :
               buts vers les DESTINATIONS ; l'accord scan / carte est controle en continu.
  idle         rien ne tourne ; la carte reste la carte courante, sur le disque.
slam_toolbox n'a pas de localisation GLOBALE : quand la pose est inconnue (demarrage, robot
deplace, decalage detecte), elle est cherchee par relocalize.py sur l'image de la carte
(rotations sur place si plusieurs poses se ressemblent), puis donnee a slam_toolbox.
map_server et AMCL ne servent plus.

SAUVEGARDE (rien ne doit se perdre) : graphe + image + meta toutes les `save_period_s` quand
le robot a bouge, et a chaque evenement (gate atteinte ou abandonnee, destination, sous-mode,
arret, sortie du mode NAV). L'historique (events.jsonl) recoit chaque evenement a l'instant.

UNE SEULE PORTE D'ENTREE : le service ~/command (NavCommand), pour la manette comme pour le
MCP. L'etat part sur /navigation/state (NavState, latche).

DIAGNOSTIC `navigation` DEPENDANT DES CAPTEURS : la liste des capteurs EXIGES vient du bloc
`sensors:` du robot (nav_params.required_sensors). Sans lidar, la navigation est declaree
impossible (ERROR), avec la raison.
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
from rclpy.serialization import deserialize_message
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
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from slam_toolbox.srv import SerializePoseGraph
from tf2_ros import Buffer, TransformListener

from bamboo_interfaces.msg import NavState, RobotMode, TeleopState
from bamboo_interfaces.srv import NavCommand
from bamboo_navigation import frontier, map_io
from bamboo_navigation import relocalize as rl

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
        # --- sauvegarde : periodique (si le robot a bouge) + sur evenement ---
        self.save_period_s = float(d("save_period_s", 60.0).value)
        # --- relocalisation (relocalize.py) et controle carte = realite ---
        # Seuils de SCORE (accord du scan avec la carte, 0..1) : a caler a l'essai sur des
        # scans reels (G3/G5) ; hors robot, la vraie pose score 0,7-0,95.
        self.reloc_accept = float(d("reloc_accept", 0.55).value)
        self.reloc_margin = float(d("reloc_margin", 0.10).value)
        self.reloc_local_r = float(d("reloc_local_radius_m", 1.5).value)
        self.reloc_turns = int(d("reloc_turns", 3).value)
        self.reloc_turn_speed = float(d("reloc_turn_speed", 0.5).value)
        # Robot non reconnu sur la carte courante, en navigation : autres cartes essayees (au
        # plus ce nombre, les plus recentes ; ~1 a 2 s chacune sur le RPi4).
        self.reloc_other_maps_max = int(d("reloc_other_maps_max", 10).value)
        # Score sous lequel une pose trouvee sur la carte COURANTE ne suffit pas : les autres
        # cartes sont aussi essayees, et une nettement meilleure (+ reloc_margin) l'emporte.
        # Essai du 2026-10-13 : 0,63 accepte sur la grande carte alors que la petite donnait
        # 0,94 au meme endroit.
        self.reloc_good = float(d("reloc_good", 0.80).value)
        self._lastLocScore = None
        self._lastOtherScore = None
        # Choix de carte a la manette (+ / -) : applique apres ce delai sans nouvel appui.
        self.map_select_delay_s = float(d("map_select_delay_s", 2.0).value)
        self._mapSel, self._mapSelT = None, 0.0
        self._forcedMap = False          # carte FORCEE : pas de recherche sur les autres
        self._wantSub = None             # sous-mode demande (exploration | navigation)
        self.range_max = float(d("lidar_range_max_m", 8.0).value)
        self.match_period_s = float(d("match_period_s", 2.0).value)
        # Controle CONTINU en navigation : CALES le 2026-10-13 sur une session reelle (8 buts,
        # 7 atteints, robot bien localise) : accord 0,33 a 0,45 en roulant (meubles, scan
        # partiel, carte sauvee il y a une minute) -- 0,45 alertait sans cesse.
        self.match_ok = float(d("match_ok", 0.30).value)
        self.match_lost = float(d("match_lost", 0.15).value)
        self.match_lost_s = float(d("match_lost_s", 5.0).value)
        # SAUT de la correction map->odom entre deux controles (2 s) au-dela duquel la pose
        # est jugee sur la carte sauvee (exploration, cf. _jumpCheck).
        self.jump_m = float(d("jump_m", 0.5).value)
        self.jump_rad = float(d("jump_rad", 0.35).value)
        # --- exploration AUTOMATIQUE (gates) : reglages, cf. _autoTick ---
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
        self.auto_recal_min_interval_s = float(d("auto_recal_min_interval_s", 600.0).value)
        # Gyro qui tourne roues immobiles au-dela de ce seuil : biais derive -> recalibration
        # forcee (0,01 rad/s = 34 deg/min ; la derive normale est de 0,12 deg/min).
        self.auto_gyro_drift_rad_s = float(d("auto_gyro_drift_rad_s", 0.01).value)
        odom_in_topic = str(d("odom_in_topic", "/odom/unfiltered").value)
        teleop_topic = str(d("teleop_topic", "/cmd_vel_teleop").value)
        battery_topic = str(d("battery_topic", "/battery").value)
        mode_topic = str(d("mode_topic", "/bamboo/mode").value)
        teleop_state_topic = str(d("teleop_state_topic", "/teleop/state").value)
        cmd_vel_nav_topic = str(d("cmd_vel_nav_topic", "/cmd_vel_nav").value)

        self.submode = "idle"
        self.slam = None              # sous-processus slam_toolbox
        self.slamMode = None          # None | mapping | localization
        self._nav2Up = False
        self._nav2StateCli = None     # /bt_navigator/get_state (etat reel de Nav2)
        self.goal_name, self.goal_status = "", ""
        self._goalInfo = {}           # {"kind": dest|home|gate, ...} du but en cours
        self.distance, self.recoveries = math.nan, 0
        self.detail = "pret"
        self._goal = None
        self._goalSeq = 0
        self._cancelPending = False
        self._lock = threading.Lock()
        self._autoTickLock = threading.Lock()
        self._last = {"scan": None, "imu": None, "odom": None}
        self._scanRaw, self._odomRaw = None, None
        self._imuRaw, self._wheelRaw = None, None
        self._laser = {}              # frame du lidar -> pose (x, y, cap) sur le robot
        # Carte courante, sa meta, et son image prete a scorer (relocalisation, controle).
        self.store = map_io.MapStore(self.maps_dir)
        self.map_name, self.meta = "", None
        self._lmap, self._lmapKey = None, None
        self.sel = {"gate": None, "dest": None}
        self._lastSaveT = -1e9           # derniere sauvegarde (dedoublonnage, periodique)
        self._lastSavePose = None
        # Localisation : inconnue | ok | douteuse | relocalisation | perdue.
        self.loc = "inconnue"
        self.match = math.nan
        self._badT = None                # debut d'un accord scan/carte sous match_lost
        self._doubtLogged = False
        self._relocBusy = False
        self._m2o = None                 # derniere correction map->odom jugee saine
        self._lastRecalT = -1e9          # derniere recalibration du gyro (monotonic)
        # Etat de l'exploration automatique : off | running | paused | homing | finishing |
        # done.
        self.auto = "off"
        self._autoResumeTo = "running"   # etat a reprendre apres une pause manette
        self._autoT0 = 0.0
        self._autoHome = None            # pose de depart de la SESSION (map)
        self._autoTarget = None          # (x, y) du but en cours
        self._autoGate = None            # gate (dict de meta) visee en ce moment
        self._autoGoalT = 0.0
        self._autoOwnCancel = False      # annulation par NOUS (pause) : pas un echec
        self._autoSecond = False         # seconde chance deja donnee ?
        self._autoProgT = 0.0            # dernier progres du but (distance restante en baisse)
        self._autoBestD = math.inf
        self._autoPrio = None            # gate DESIGNEE (numero), visee avant les autres
        # X de la manette (2026-10-13) : aller a la gate designee PUIS s'arreter.
        self._autoOnce = False
        # RONDE des destinations (A en navigation, 2026-10-13) : off | running | paused.
        self.patrol = "off"
        self._patrolOrder, self._patrolIdx, self._patrolSent = [], 0, False
        self._autoNoFront = 0
        self._autoHomeTries = 0
        self._autoFronts = 0
        self._autoKnown = math.nan
        self._gatesSig = None            # signature des gates (sauvegarde de meta au changement)
        self._autoClearT = -1e9          # dernier vidage des costmaps (echecs en serie)
        self._teleopT = 0.0              # dernier ordre NON NUL de la manette
        self._battV = math.nan           # mediane glissante (cf. _onBattery)
        self._battPct = math.nan         # etat de charge AU REPOS (0..100), du driver
        self._battHist = collections.deque()
        self._robotMode = ""
        self._lastRobotMode = ""
        self._armed = None               # traction armee a la manette ? None = inconnu (simu)
        self._objSig = None

        # TF ecoutee SEULEMENT hors idle (cf. _tfListen) : un TransformListener Python sur tout
        # /tf, sous MultiThreadedExecutor, coutait a lui seul une bonne part des ~30 % d'un
        # coeur que prenait le superviseur au repos (essai Nb, 2026-10-10).
        self.tf = Buffer()
        self.tfl = None
        # Fraicheur seule : abonnements BRUTS (raw=True, sans deserialisation) portes par un
        # noeud SATELLITE tourne par un executeur MONO-fil dans son propre thread. Sous le
        # MultiThreadedExecutor du superviseur, chacun des ~50 messages/s reveillait le pool
        # de fils Python : ~30 % d'un coeur du RPi4 au repos (essai Nb, 2026-10-10). Le
        # DERNIER scan et la DERNIERE odometrie sont gardes bruts, et deserialises seulement
        # quand on s'en sert (relocalisation, controle de la carte : toutes les 2 s au plus).
        # use_global_arguments=False : sinon le `__node:=nav_supervisor` du launch le renomme
        # lui aussi, et le graphe montre deux /nav_supervisor (services de parametres en double).
        self._watch = rclpy.create_node("nav_supervisor_watch", use_global_arguments=False)
        self._watch.create_subscription(LaserScan, self.scan_topic, self._onScanRaw,
                                        qos_profile_sensor_data, raw=True)
        self._watch.create_subscription(Imu, self.imu_topic, self._onImuRaw,
                                        qos_profile_sensor_data, raw=True)
        self._watch.create_subscription(Odometry, self.odom_topic, self._onOdomRaw, 1,
                                        raw=True)
        # Odometrie des ROUES seules (avant l'EKF) : immobilite sans le biais du gyro.
        self._watch.create_subscription(Odometry, odom_in_topic,
                                        lambda raw: setattr(self, "_wheelRaw", raw), 1,
                                        raw=True)
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
        # et TOUTES les gates passeraient en echec (essai E1, 2026-10-11).
        self._watch.create_subscription(
            TeleopState, teleop_state_topic, lambda m: setattr(self, "_armed", bool(m.armed)),
            latched_in)
        self._watchEx = SingleThreadedExecutor()
        self._watchEx.add_node(self._watch)
        threading.Thread(target=self._watchEx.spin, name="nav_watch", daemon=True).start()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # La carte est DEMANDEE quand il le faut (GetMap), pas suivie en continu : s'abonner a
        # /map faisait deserialiser toute la grille en Python a chaque mise a jour de
        # slam_toolbox (2 s) (2026-10-11).
        self.mapCli = self.create_client(GetMap, "/slam_toolbox/dynamic_map",
                                         callback_group=self._cbg)
        self.serCli = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map",
                                         callback_group=self._cbg)
        self.pubState = self.create_publisher(NavState, "/navigation/state", latched)
        # Pose trouvee par la relocalisation, donnee a slam_toolbox (mode localisation).
        self.pubInit = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        # AUTORISATION de la voie navigation du multiplexeur : ouverte SEULEMENT pendant un
        # but actif (ou une rotation de relocalisation). A l'arret, Nav2 met ~1 s a s'eteindre
        # et ses manoeuvres de reprise publient en direct : sans ce verrou, soubresauts (essai
        # E1, 2026-10-11).
        self.pubEnable = self.create_publisher(
            Bool, str(d("enable_topic", "/navigation/enable").value), latched)
        self._navEnabled = None
        self._setNavEnable(False)
        # Rotations sur place de la relocalisation : sur la MEME voie que Nav2 (cmd_vel_nav,
        # sous le verrou ci-dessus) ; la manette garde la priorite dans le multiplexeur.
        self.pubCmd = self.create_publisher(Twist, cmd_vel_nav_topic, 1)
        # Pour la MINI-CARTE du HUD (overlay_node) et le MCP : pose du robot dans map (4 Hz,
        # hors idle), depart, gates du moment, et le detail des objets de la carte (JSON).
        self.pubPose = self.create_publisher(PoseStamped, "/navigation/robot_pose", 1)
        self.pubHome = self.create_publisher(PoseStamped, "/navigation/home", latched)
        self.pubFronts = self.create_publisher(PoseArray, "/navigation/frontiers", latched)
        self.pubLog = self.create_publisher(String, "/navigation/frontier_log", latched)
        self.pubObjs = self.create_publisher(String, "/navigation/objects", latched)
        # APERCU pour la mini-carte du HUD : la carte choisie au + / -, ou la carte courante en
        # idle (slam arrete, donc pas de /map). Sur un sujet A PART : /map nourrit aussi la
        # costmap globale de Nav2. Grille vide = pas d'apercu (demande utilisateur 2026-10-13).
        self.pubPreview = self.create_publisher(OccupancyGrid, "/navigation/map_preview",
                                                latched)
        self._previewName = ""
        self.create_timer(0.25, self._publishPose, callback_group=self._cbg)

        self.navMgr = self.create_client(ManageLifecycleNodes,
                                         "/lifecycle_manager_navigation/manage_nodes",
                                         callback_group=self._cbg)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose",
                                callback_group=self._cbg)
        # Pour verifier qu'une gate est ATTEIGNABLE avant d'y envoyer le robot.
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
        self.create_timer(1.0, self._patrolTick, callback_group=self._cbg)
        self.create_timer(5.0, self._saveTick, callback_group=self._cbg)
        self.create_timer(0.5, self._mapSelTick, callback_group=self._cbg)
        self.create_timer(self.match_period_s, self._locTick, callback_group=self._cbg)
        self._useMap(self._initialMap())
        self.get_logger().info("superviseur de navigation : cartes dans %s, carte courante %s, "
                               "capteurs exiges %s" % (self.maps_dir, self.map_name or "aucune",
                                                       self.required or "aucun"))

    # ------------------------------------------------------------ capteurs
    def _seen(self, k):
        self._last[k] = time.monotonic()

    def _onScanRaw(self, raw):
        self._scanRaw = raw
        self._seen("scan")

    def _onImuRaw(self, raw):
        self._imuRaw = raw
        self._seen("imu")

    def _onOdomRaw(self, raw):
        self._odomRaw = raw
        self._seen("odom")

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

    def _fresh(self, k):
        t = self._last.get(k)
        return t is not None and time.monotonic() - t <= self.stale_s

    def _odomPose(self):
        """(x, y, cap) de la derniere odometrie (EKF), ou None."""
        raw = self._odomRaw
        if raw is None:
            return None
        p = deserialize_message(raw, Odometry).pose.pose
        return (p.position.x, p.position.y,
                map_io.yaw_of(p.orientation.z, p.orientation.w, p.orientation.x,
                              p.orientation.y))

    def _scanPts(self, after=None, timeout=2.0):
        """Points du dernier scan dans le repere du robot, et l'horodatage du scan ; avec
        `after`, attend un scan ARRIVE apres cet instant (monotonic). (None, None) sinon."""
        t0 = time.monotonic()
        while after is not None and (self._last["scan"] or 0.0) <= after:
            if time.monotonic() - t0 > timeout:
                return None, None
            time.sleep(0.05)
        raw = self._scanRaw
        if raw is None:
            return None, None
        s = deserialize_message(raw, LaserScan)
        laser = self._laserPose(s.header.frame_id)
        if laser is None:
            return None, None
        pts = rl.scan_points(s.ranges, s.angle_min, s.angle_increment,
                             max(0.05, float(s.range_min)),
                             min(self.range_max, float(s.range_max) or self.range_max), laser)
        return (pts if len(pts) >= 20 else None), s.header.stamp

    def _laserPose(self, frame):
        """Pose du lidar sur le robot (TF statique, gardee une fois lue)."""
        if frame in self._laser:
            return self._laser[frame]
        self._tfListen(True)
        try:
            t = self.tf.lookup_transform("base_footprint", frame, Time(),
                                         timeout=Duration(seconds=1.0))
        except Exception:                                   # noqa: BLE001
            return None
        q = t.transform.rotation
        p = (t.transform.translation.x, t.transform.translation.y,
             map_io.yaw_of(q.z, q.w, q.x, q.y))
        self._laser[frame] = p
        return p

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
            miss.append("TF map->odom (slam_toolbox %s)" % (self.slamMode or "arrete"))
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

    def _nav2(self, on):
        """Nav2 actif ou au repos. Au repos en exploration manuelle : ses costmaps et son
        controleur coutaient ~50 % d'un coeur du RPi4 pour rien (2026-10-11)."""
        # L'ETAT REEL de Nav2 d'abord, pas seulement notre drapeau : un demarrage qui depasse
        # le delai (RPi charge) aboutit quand meme, et Nav2 deja ACTIF refuse un nouveau
        # STARTUP (« transition is not registered from state active ») -- essai du
        # 2026-10-13 : ronde refusee, « Nav2 refuse », alors que Nav2 tournait.
        if on and not self._nav2Up:
            if self._nav2Active():
                self._nav2Up = True
                return True, "deja actif"
            ok, why = self._manage(self.navMgr, STARTUP, timeout=60.0)
            if not ok and self._nav2Active():
                ok, why = True, "actif (demarrage plus long que prevu)"
            self._nav2Up = ok
            return ok, why
        if not on and (self._nav2Up or self._nav2Active()):
            self._nav2Up = False
            return self._manage(self.navMgr, RESET)
        return True, "deja"

    def _nav2Active(self):
        """bt_navigator est-il ACTIF (cycle de vie) ? False si on ne sait pas."""
        if self._nav2StateCli is None:
            self._nav2StateCli = self.create_client(GetState, "/bt_navigator/get_state",
                                                    callback_group=self._cbg)
        cli = self._nav2StateCli
        if not cli.wait_for_service(timeout_sec=1.0):
            return False
        fut = cli.call_async(GetState.Request())
        if not self._wait(fut, 3.0):
            return False
        return fut.result().current_state.id == State.PRIMARY_STATE_ACTIVE

    def _slamAlive(self):
        return self.slam is not None and self.slam.poll() is None

    def _startSlam(self, mode, graph=None, pose=None):
        """slam_toolbox en `mapping` (cartographie) ou `localization`. Avec `graph` (chemin
        sans extension) et `pose` : graphe recharge, robot a cette pose (map_start_pose) --
        c'est ce qui COMPLETE une carte, ou s'y localise."""
        self._stopSlam()
        exe = "localization_slam_toolbox_node" if mode == "localization" \
            else "async_slam_toolbox_node"
        cmd = ["ros2", "run", "slam_toolbox", exe, "--ros-args",
               "--params-file", self.slam_params, "-r", "map:=%s" % self.map_topic,
               "-p", "mode:=%s" % mode]
        if graph:
            cmd += ["-p", "map_file_name:=%s" % graph,
                    "-p", "map_start_pose:=[%.4f, %.4f, %.4f]" % tuple(pose)]
        self.tf.clear()                # sinon l'ANCIEN map->odom reste quelques secondes
        self._m2o = None               # nouveau repere : rien a comparer
        self.slam = subprocess.Popen(cmd, start_new_session=True)
        self.slamMode = mode
        t = time.monotonic()
        while time.monotonic() - t < 25.0:
            if self._tfOk("map", "odom"):
                return True, "slam_toolbox (%s) actif" % mode
            if self.slam.poll() is not None:
                self.slamMode = None
                return False, "slam_toolbox s'est arrete (code %s)" % self.slam.returncode
            time.sleep(0.3)
        return True, "slam_toolbox (%s) lance (pas encore de map->odom)" % mode

    def _stopSlam(self):
        self.slamMode = None
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

    # ------------------------------------------------------------ carte courante
    def _initialMap(self):
        cur = self.store.current()
        if cur:
            return cur
        if self.default_map and self.store.exists(self.default_map):
            return self.default_map
        maps = [m["name"] for m in self.store.list() if m["kind"] == "carte"]
        return maps[0] if maps else ""

    def _useMap(self, name):
        self.map_name = name or ""
        self.meta = self.store.load_meta(name) if name else None
        if name:
            self.store.set_current(name)
        self.sel = {"gate": None, "dest": None}
        self._lmap, self._lmapKey = None, None
        self._gatesSig = None

    def _event(self, kind, **kw):
        if not self.map_name:
            return
        try:
            self.store.append_event(self.map_name, kind, **kw)
        except OSError as e:
            self.get_logger().warn("historique %s non ecrit : %s" % (self.map_name, e))

    def _saveMeta(self):
        if self.map_name and self.meta is not None:
            try:
                self.store.save_meta(self.map_name, self.meta)
            except OSError as e:
                self.get_logger().warn("meta %s non ecrite : %s" % (self.map_name, e))

    def _getMap(self, timeout=10.0):
        if not self.mapCli.wait_for_service(timeout_sec=2.0):
            return None
        fut = self.mapCli.call_async(GetMap.Request())
        if not self._wait(fut, timeout):
            return None
        return fut.result().map

    def _serialize(self):
        """Graphe de poses de slam_toolbox -> dossier de la carte (via un temporaire)."""
        if not self.serCli.wait_for_service(timeout_sec=2.0):
            return False
        req = SerializePoseGraph.Request()
        req.filename = self.store.posegraph_tmp(self.map_name)
        fut = self.serCli.call_async(req)
        if not self._wait(fut, 30.0) or fut.result().result != 0:
            return False
        return self.store.commit_posegraph(self.map_name)

    def _saveMap(self, why, force=False):
        """Sauve la carte COURANTE : graphe + image en cartographie (en localisation le graphe
        ne change pas), meta (derniere pose) toujours, et l'evenement. Dedoublonnee sur 10 s
        (plusieurs causes d'arret arrivent ensemble), sauf `force`."""
        if not self.map_name:
            return False, "aucune carte courante"
        now = time.monotonic()
        if not force and now - self._lastSaveT < 10.0:
            return True, "carte %s deja sauvee" % self.map_name
        parts, known = [], math.nan
        if self.slamMode == "mapping" and self._slamAlive():
            # JAMAIS une carte douteuse par-dessus une bonne (essai G1, 2026-10-13 : apres un
            # saut de la pose, les sauvegardes periodiques avaient ecrase la derniere bonne
            # carte). Controle du saut juste avant d'ecrire, en plus de celui des 2 s.
            if self.loc != "ok" or self._jumpCheck()[0] == "saut":
                self._event("sauvegarde_refusee", raison=why, localisation=self.loc)
                return False, "carte %s NON sauvee : localisation %s" % (
                    self.map_name, self.loc)
            self.store.backup(self.map_name)
            m = self._getMap()
            if m is None:
                parts.append("image NON sauvee (GetMap muet)")
            else:
                q = m.info.origin.orientation
                self.store.write_grid(self.map_name, m.info.width, m.info.height,
                                      m.info.resolution,
                                      (m.info.origin.position.x, m.info.origin.position.y,
                                       map_io.yaw_of(q.z, q.w, q.x, q.y)), m.data)
                known, _occ = map_io.map_stats(m.info.width, m.info.height, m.data)
                self._lmap = None             # l'image a change
                parts.append("image %dx%d" % (m.info.width, m.info.height))
            parts.append("graphe" if self._serialize() else "graphe NON sauve")
        pose = self._robotPose() if self.slamMode else None
        if pose is not None and self.meta is not None:
            self.meta["last_pose"] = [round(v, 3) for v in pose]
        self._saveMeta()
        self._lastSaveT, self._lastSavePose = now, pose
        msg = "carte %s sauvee (%s)%s" % (
            self.map_name, ", ".join(parts) or "meta",
            "" if math.isnan(known) else ", %.0f %% connue" % (100 * known))
        self._event("sauvegarde", raison=why, detail=", ".join(parts) or "meta")
        self.get_logger().info("%s : %s" % (why, msg))
        return True, msg

    def _saveTick(self):
        """Sauvegarde PERIODIQUE en cartographie, si le robot a bouge depuis la derniere."""
        if self.slamMode != "mapping" or not self._slamAlive():
            return
        if time.monotonic() - self._lastSaveT < self.save_period_s:
            return
        pose, last = self._robotPose(), self._lastSavePose
        if pose is not None and last is not None and \
                math.hypot(pose[0] - last[0], pose[1] - last[1]) < 0.2 and \
                abs(rl.angdiff(pose[2], last[2])) < 0.3:
            return
        if not self._lock.acquire(blocking=False):
            return                         # une commande est en cours : au prochain tour
        try:
            self._saveMap("periodique", force=True)
        finally:
            self._lock.release()

    def _mapModel(self):
        """Image de la carte courante prete a scorer (relocalisation, controle), ou None."""
        key = (self.map_name, self._lastSaveT)
        if self._lmap is not None and self._lmapKey == key:
            return self._lmap
        if not self.map_name or not self.store.has_image(self.map_name):
            return None
        w, h, res, origin, data = map_io.read_map(self.store.path(self.map_name, "map.yaml"))
        self._lmap = rl.LikelihoodMap.from_occupancy(data, w, h, res, origin)
        self._lmapKey = key
        return self._lmap

    def _otherIfBetter(self):
        """Pose trouvee sur la carte courante avec un score MOYEN (< reloc_good) : une autre
        carte la reconnait-elle NETTEMENT mieux ? (nom, pose, message) ou None. Jamais pour une
        carte forcee au + / -."""
        score = self._lastLocScore
        if self._forcedMap or score is None or score >= self.reloc_good:
            return None
        other, pose, msg = self._locateOtherMaps()
        if other is None or self._lastOtherScore is None or \
                self._lastOtherScore < score + self.reloc_margin:
            return None
        self.get_logger().info("relocalisation : %s mieux que la carte courante (%.2f contre "
                               "%.2f)" % (other, self._lastOtherScore, score))
        return other, pose, "%s (carte courante : %.2f seulement)" % (msg, score)

    def _locateOtherMaps(self):
        """Le robot n'est pas reconnu sur la carte courante : est-il sur une AUTRE carte ?
        (demande utilisateur 2026-10-13, sous-mode navigation : robot emmene dans une autre
        piece deja cartographiee). Les cartes a graphe de poses, la plus recente d'abord (au
        plus `reloc_other_maps_max`), recherche GLOBALE sur chacune, sans rotation ; choix par
        relocalize.pick_map. Rend (nom, pose, message) ou (None, None, raison)."""
        pts, _st = self._scanPts()
        if pts is None:
            return None, None, "pas de scan lidar exploitable"
        names = [m["name"] for m in self.store.list()
                 if m["kind"] == "carte" and m["name"] != self.map_name and
                 self.store.has_image(m["name"])][:self.reloc_other_maps_max]
        if not names:
            return None, None, "aucune autre carte"
        t0, results = time.monotonic(), []
        for k, name in enumerate(names):
            self.detail = "relocalisation : carte %s (%d/%d)" % (name, k + 1, len(names))
            try:
                w, h, res, origin, data = map_io.read_map(self.store.path(name, "map.yaml"))
            except (OSError, ValueError, KeyError) as e:
                self.get_logger().warn("carte %s illisible : %s" % (name, e))
                continue
            lm = rl.LikelihoodMap.from_occupancy(data, w, h, res, origin)
            ok, best, rival = rl.Relocalizer(lm, self.reloc_accept, self.reloc_margin).start(pts)
            results.append((name, ok, best, rival))
        got, why = rl.pick_map(results, self.reloc_margin)
        dt = time.monotonic() - t0
        self._event("recherche_autres_cartes", cartes=len(results), duree_s=round(dt, 1),
                    trouvee=got[0] if got else None,
                    score=round(got[1][0], 3) if got else None)
        if got is None:
            return None, None, why
        name, best = got
        self._lastOtherScore = best[0]
        return name, best[1], "robot retrouve sur la carte %s (score %.2f, %d carte(s) " \
            "essayee(s), %.1f s)" % (name, best[0], len(results), dt)

    # ------------------------------------------------------------ relocalisation
    def _rotate(self, angle):
        """Rotation sur place de `angle` rad, mesuree par l'odometrie (gyro). False si le
        stick reprend la main, sans odometrie, ou trop long."""
        o = self._odomPose()
        if o is None:
            return False
        sign = 1.0 if angle >= 0 else -1.0
        turned, last, t0 = 0.0, o[2], time.monotonic()
        limit = abs(angle) / self.reloc_turn_speed * 2.0 + 3.0
        self._setNavEnable(True)
        try:
            while turned < abs(angle) - 0.05:
                if time.monotonic() - self._teleopT < 1.0 or time.monotonic() - t0 > limit:
                    return False
                tw = Twist()
                tw.angular.z = sign * self.reloc_turn_speed
                self.pubCmd.publish(tw)
                time.sleep(0.1)
                o = self._odomPose()
                if o is None:
                    return False
                turned += abs(rl.angdiff(o[2], last))
                last = o[2]
            return True
        finally:
            for _ in range(3):
                self.pubCmd.publish(Twist())
                time.sleep(0.05)
            self._setNavEnable(False)

    def _locate(self, prior=None):
        """Ou est le robot sur la carte courante ? (pose, message). Scan + image de la carte,
        sans slam_toolbox : recherche LOCALE autour de `prior` d'abord (rapide, sure), puis
        GLOBALE ; en cas d'ambiguite, rotations de 90 degres (traction armee) et cumul."""
        lm = self._mapModel()
        if lm is None:
            return None, "carte %s sans image" % (self.map_name or "-")
        pts, _st = self._scanPts()
        if pts is None:
            return None, "pas de scan lidar exploitable (ou TF du lidar absente)"
        t0 = time.monotonic()
        r = rl.Relocalizer(lm, self.reloc_accept, self.reloc_margin)
        ok, best, rival, how = False, None, None, "globale"
        if prior is not None:
            ok, best, rival = r.start(pts, region=(prior[0], prior[1], self.reloc_local_r))
            how = "locale"
        if not ok:
            ok, best, rival = r.start(pts)
            how = "globale"
        turns = 0
        while not ok and best is not None and turns < self.reloc_turns:
            if self._armed is not True:     # rotation : traction ARMEE explicitement
                break
            o0 = self._odomPose()
            self.detail = "relocalisation : ambigue, rotation %d/%d" % (turns + 1,
                                                                        self.reloc_turns)
            if o0 is None or not self._rotate(math.pi / 2):
                break
            time.sleep(0.3)
            pts, _st = self._scanPts(after=time.monotonic())
            o1 = self._odomPose()
            if pts is None or o1 is None:
                break
            ok, best, rival = r.update(pts, rl.relative(o0, o1))
            turns += 1
        dt = time.monotonic() - t0
        info = dict(recherche=how, rotations=turns, duree_s=round(dt, 1),
                    score=round(best[0], 3) if best else None,
                    rivale=round(rival[0], 3) if rival else None)
        if not ok:
            why = ("aucune pose plausible" if best is None or best[0] < self.reloc_accept
                   else "plusieurs poses possibles (%.2f contre %.2f)" % (best[0], rival[0]))
            if self._armed is not True and turns < self.reloc_turns and best is not None:
                why += " ; armer la traction (B long) pour tourner et lever le doute"
            self._event("relocalisation_echec", raison=why, **info)
            return None, "relocalisation impossible : " + why
        self._event("relocalisation", pose=best[1], **info)
        self._lastLocScore = best[0]
        return best[1], "relocalise (%s, score %.2f, %d rotation(s), %.1f s)" % (
            how, best[0], turns, dt)

    def _sendInitialPose(self, pose):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(pose[0]), float(pose[1])
        m.pose.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
        m.pose.pose.orientation.w = math.cos(float(pose[2]) / 2.0)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05
        m.pose.covariance[35] = 0.05
        for _ in range(3):
            self.pubInit.publish(m)
            time.sleep(0.2)

    def _matchNow(self):
        """Accord du dernier scan avec l'image de la carte, a la pose de slam_toolbox."""
        lm = self._mapModel()
        pts, stamp = self._scanPts()
        if lm is None or pts is None:
            return math.nan
        pose = self._robotPose(stamp)
        if pose is None:
            return math.nan
        return lm.judge(pts, pose)

    def _tf2d(self, parent, child):
        """(x, y, cap) de la TF parent -> child, derniere connue, ou None."""
        if self.tfl is None:
            return None
        try:
            t = self.tf.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.2))
        except Exception:                                   # noqa: BLE001
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                map_io.yaw_of(q.z, q.w, q.x, q.y))

    def _jumpCheck(self):
        """SAUT de localisation en cartographie ? Essai G1 (2026-10-13) : la correction
        map->odom de slam_toolbox a saute de 4,6 m en 8 s (fausse fermeture de boucle
        probable), et la carte s'est dedoublee, tournee. Une correction normale fait quelques
        cm. Au-dela de `jump_m` / `jump_rad` depuis le controle precedent, on juge sur l'image
        SAUVEE quel repere colle au scan : le nouveau (vraie correction, acceptee) ou l'ancien
        (saut faux). Rend ("ok",) ou ("saut", pose dans l'ancien repere, score ancien, nouveau)."""
        cur = self._tf2d("map", "odom")
        if cur is None:
            return ("ok",)
        prev = self._m2o
        if prev is None or (math.hypot(cur[0] - prev[0], cur[1] - prev[1]) < self.jump_m and
                            abs(rl.angdiff(cur[2], prev[2])) < self.jump_rad):
            self._m2o = cur
            return ("ok",)
        lm = self._mapModel()
        pts, _st = self._scanPts()
        ob = self._tf2d("odom", "base_footprint")
        if lm is None or pts is None or ob is None:
            self._m2o = cur                    # rien pour juger : on accepte, et on le dit
            self._event("correction_slam", jugee=False, saut_m=math.hypot(
                cur[0] - prev[0], cur[1] - prev[1]))
            return ("ok",)
        new, old = rl.compose(cur, ob), rl.compose(prev, ob)
        s_new, s_old = lm.judge(pts, new), lm.judge(pts, old)
        jump = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        if s_new >= s_old - 0.05:
            self._m2o = cur
            self._event("correction_slam", jugee=True, saut_m=jump, score_ancien=s_old,
                        score_nouveau=s_new)
            return ("ok",)
        return ("saut", old, s_old, s_new)

    def _locTick(self):
        """CONTROLE CONTINU. En navigation : la carte correspond-elle a ce que voit le lidar ?
        Bon accord : rien. Accord moyen : « carte modifiee ? » (meuble deplace, porte...).
        Accord mauvais pendant `match_lost_s` : le robot est PERDU (souleve, glissade) ->
        pause du but, relocalisation, reprise. En exploration : SAUT de la pose de slam ->
        retour en arriere sur le dernier graphe sain (_rollbackJob)."""
        if self._relocBusy:
            return
        if self.submode == "exploration" and self.slamMode == "mapping":
            r = self._jumpCheck()
            if r[0] == "saut":
                self.get_logger().warn(
                    "exploration : SAUT de localisation (accord %.2f dans l'ancien repere, "
                    "%.2f dans le nouveau) -> retour au dernier graphe sain" % (r[2], r[3]))
                self._relocBusy = True
                self.loc = "relocalisation"
                threading.Thread(target=self._rollbackJob, args=(r[1], r[2], r[3]),
                                 daemon=True, name="nav_rollback").start()
            return
        if self.submode != "navigation" or self.slamMode != "localization":
            return
        s = self._matchNow()
        self.match = s
        if math.isnan(s):
            return
        now = time.monotonic()
        if s >= self.match_ok:
            if self.loc != "ok":
                self._event("localisation_ok", score=s)
            self.loc, self._badT, self._doubtLogged = "ok", None, False
        elif s >= self.match_lost:
            self.loc, self._badT = "douteuse", None
            if not self._doubtLogged:
                self._doubtLogged = True
                self._event("carte_douteuse", score=s)
                self.get_logger().warn("navigation : accord scan/carte %.2f -- carte modifiee ?"
                                       % s)
        else:
            if self._badT is None:
                self._badT = now
            elif now - self._badT > self.match_lost_s:
                self.get_logger().warn("navigation : robot PERDU (accord %.2f depuis %.0f s), "
                                       "relocalisation" % (s, now - self._badT))
                self._event("robot_perdu", score=s)
                self._relocStart(False, "decalage carte / lidar")

    def _relocStart(self, global_only, why):
        if self._relocBusy:
            return False, "relocalisation deja en cours"
        self._relocBusy = True
        threading.Thread(target=self._relocJob, args=(global_only, why), daemon=True,
                         name="nav_reloc").start()
        return True, "relocalisation lancee (%s)" % why

    def _relocJob(self, global_only, why):
        """Pause du but, relocalisation, pose donnee a slam_toolbox, reprise du but."""
        resume = None
        try:
            with self._lock:
                if self.goal_status == "active" and self._goalInfo.get("kind") in \
                        ("dest", "home"):
                    resume = dict(self._goalInfo["wp"], kind=self._goalInfo["kind"])
                self._cancelGoal()
                self.loc = "relocalisation"
                self.detail = "relocalisation (%s)" % why
                prior = None if global_only else self._robotPose()
            # HORS du verrou : la recherche et les rotations durent jusqu'a ~15 s, et l'etat
            # doit continuer de partir (HUD), une commande (annuler) de passer.
            pose, msg = self._locate(prior)
            better = self._otherIfBetter() if pose is not None and \
                self.submode == "navigation" else None
            if (pose is None or better is not None) and self.submode == "navigation":
                # Robot emmene dans une autre piece : peut-etre sur une AUTRE carte (ou une
                # autre carte le reconnait nettement mieux). Le but en cours n'est pas repris
                # (il etait dans le repere de l'ancienne carte).
                other, opose, omsg = better if better is not None else \
                    self._locateOtherMaps()
                if other is not None:
                    with self._lock:
                        self._event("carte_quittee", vers=other, raison="robot perdu")
                        self._stopSlam()
                        self._useMap(other)
                        self._event("chargee", raison="robot retrouve sur cette carte")
                        ok, why2 = self._startSlam("localization",
                                                   self.store.posegraph_base(other), opose)
                        self.loc = "ok" if ok else "perdue"
                        self.detail = "%s ; %s" % (omsg, why2)
                        self.get_logger().warn(self.detail)
                    return
                msg = "%s ; %s" % (msg, omsg)
            if pose is None:
                self.loc, self.detail = "perdue", msg
                self.get_logger().warn(msg)
                return
            self._sendInitialPose(pose)
            time.sleep(3.0)                    # slam_toolbox recale sur la pose donnee
            s = self._matchNow()
            self.match = s
            self.loc = "ok" if s == s and s >= self.match_ok else "douteuse"
            self.detail = "%s ; accord %.2f" % (msg, s)
            self.get_logger().info(self.detail)
            if resume is not None and self.loc == "ok":
                with self._lock:
                    kind = resume.pop("kind")
                    self._goto(resume, kind)
                    self.detail += " ; reprise vers %s" % resume["name"]
        finally:
            self._badT = None
            self._relocBusy = False

    def _rollbackJob(self, pose_old, s_old, s_new):
        """RETOUR EN ARRIERE apres un saut de localisation en exploration : l'exploration est
        mise en pause, slam_toolbox repart du DERNIER GRAPHE SAUVE (sauve avant le saut : les
        sauvegardes sont refusees des que la localisation est douteuse), a la pose du robot
        dans l'ancien repere, affinee par une relocalisation LOCALE sur l'image sauvee. Puis
        l'exploration reprend d'elle-meme (_autoTick attend la fin de ce travail)."""
        name = self.map_name
        try:
            with self._lock:
                self._event("saut_localisation", pose_ancien_repere=pose_old,
                            score_ancien=s_old, score_nouveau=s_new)
                if self.auto in ("running", "homing"):
                    self._autoResumeTo, self.auto = self.auto, "paused"
                    self._autoOwnCancel = True
                    self._gateSet(self._autoGate, "ouverte", "saut de localisation")
                    self._autoGate, self._autoTarget = None, None
                self._cancelGoal()
                if not self.store.has_posegraph(name):
                    self.loc = "perdue"
                    self.detail = "saut de localisation et aucun graphe sauve : nouvelle carte"
                    return
                self._stopSlam()
            pose, msg = self._locate(pose_old)
            if pose is None:
                # Pose de l'ancien repere telle quelle : l'odometrie n'a derive que de
                # quelques secondes depuis le dernier controle.
                pose, msg = pose_old, "pose de l'ancien repere (%s)" % msg
            with self._lock:
                ok, why = self._startSlam("mapping", self.store.posegraph_base(name), pose)
                self.loc = "ok" if ok else "perdue"
                self.detail = "retour au dernier graphe sain : %s ; %s" % (msg, why)
                self._event("retour_arriere", ok=ok, pose=pose, detail=msg)
                self.get_logger().warn(self.detail)
        finally:
            self._relocBusy = False

    # ------------------------------------------------------------ sous-modes
    def _enterMapping(self, new=False):
        """slam_toolbox en cartographie : sur la carte courante (graphe recharge a la pose du
        robot), ou sur une carte VIERGE (`new`, pas de carte, ou carte sans graphe)."""
        self._tfListen(True)
        if self.slamMode == "mapping" and self._slamAlive() and not new:
            return True, "cartographie deja active (%s)" % self.map_name
        if self.slamMode:
            self._saveMap("changement de sous-mode", force=True)
        name = self.map_name
        fresh = new or not name or not self.store.has_posegraph(name)
        if fresh:
            # Dossier deja cree et encore vide (aucune image) : on le garde ; sinon un neuf.
            if new or not name or self.store.has_image(name):
                self._useMap(self.store.create())
            ok, why = self._startSlam("mapping")
            if ok:
                self.loc, self.match = "ok", math.nan
                self._event("cartographie", reprise=False)
            return ok, "nouvelle carte %s : %s" % (self.map_name, why)
        if self.slamMode == "localization" and self._slamAlive():
            pose, how = self._robotPose(), "a la pose courante"
            if pose is None:
                return False, "pose inconnue (TF map->base_footprint)"
        else:
            pose, how = self._locate(self.meta.get("last_pose") if self.meta else None)
            if pose is None:
                # Pas sur la carte courante : peut-etre sur une AUTRE carte, qu'on completera.
                if self._forcedMap:        # carte choisie au + / - : elle, et elle seule
                    other, pose, how2 = None, None, "carte forcee : autres cartes non essayees"
                else:
                    other, pose, how2 = self._locateOtherMaps()
                if other is None:
                    return False, "carte %s : %s ; %s -- ou nouvelle carte (+ long)" % (
                        name, how, how2)
                self._event("carte_quittee", vers=other, raison="robot sur une autre carte")
                self._useMap(other)
                self._event("chargee", raison="robot retrouve sur cette carte")
                name, how = other, how2
            else:
                better = self._otherIfBetter()
                if better is not None:
                    other, pose, how = better
                    self._event("carte_quittee", vers=other, raison="autre carte meilleure")
                    self._useMap(other)
                    self._event("chargee", raison="robot mieux reconnu sur cette carte")
                    name = other
        ok, why = self._startSlam("mapping", self.store.posegraph_base(name), pose)
        if ok:
            self.loc, self.match = "ok", math.nan
            self._event("cartographie", reprise=True, pose=pose)
        return ok, "carte %s completee (%s) : %s" % (name, how, why)

    def _enterLocalization(self):
        """slam_toolbox en localisation sur le graphe de la carte courante."""
        self._tfListen(True)
        name = self.map_name
        if not name:
            return False, "aucune carte : explorer d'abord"
        if self.slamMode == "localization" and self._slamAlive():
            return True, "deja localise sur %s" % name
        if self.slamMode == "mapping" and self._slamAlive():
            pose, how = self._robotPose(), "pose courante"
            self._saveMap("passage en navigation", force=True)
            if pose is None:
                return False, "pose inconnue (TF map->base_footprint)"
        else:
            pose, how = None, ""
        if not self.store.has_posegraph(name):
            return False, ("carte %s sans graphe de poses (image seule) : la recartographier "
                           "(exploration)" % name)
        if pose is None:
            pose, how = self._locate(self.meta.get("last_pose") if self.meta else None)
            if pose is None:
                # Pas sur la carte courante : peut-etre sur une AUTRE carte enregistree.
                if self._forcedMap:        # carte choisie au + / - : elle, et elle seule
                    other, pose, how2 = None, None, "carte forcee : autres cartes non essayees"
                else:
                    other, pose, how2 = self._locateOtherMaps()
                if other is None:
                    return False, "%s ; %s" % (how, how2)
                self._event("carte_quittee", vers=other, raison="robot sur une autre carte")
                self._useMap(other)
                self._event("chargee", raison="robot retrouve sur cette carte")
                name, how = other, how2
            else:
                better = self._otherIfBetter()
                if better is not None:
                    other, pose, how = better
                    self._event("carte_quittee", vers=other, raison="autre carte meilleure")
                    self._useMap(other)
                    self._event("chargee", raison="robot mieux reconnu sur cette carte")
                    name = other
        ok, why = self._startSlam("localization", self.store.posegraph_base(name), pose)
        if ok:
            self.loc, self.match, self._badT = "ok", math.nan, None
            self._event("localisation", pose=pose, comment=how)
        return ok, "navigation sur %s (%s) : %s" % (name, how, why)

    def setSubmode(self, mode, arg=""):
        ok, why = self._setSubmode(mode, arg)
        if not ok and self.submode == "idle" and not self.slamMode:
            self._tfListen(False)          # refus depuis idle : la TF n'a plus a etre ecoutee
        return ok, why

    def _setSubmode(self, mode, arg):
        if mode not in ("idle", "exploration", "navigation"):
            return False, "sous-mode inconnu : %s" % mode
        if mode == self.submode and not arg:
            return True, "deja en %s (%s)" % (mode, self.map_name or "aucune carte")
        if mode != "exploration" and self.auto != "off":
            self._autoStop("sous-mode %s" % mode, nav_reset=False)
        if self.patrol != "off":
            self._patrolStop("sous-mode %s" % mode)
        self._cancelGoal()
        if mode == "idle":
            if self.slamMode:
                self._saveMap("sous-mode idle", force=True)
            self._nav2(False)
            self._stopSlam()
            self._tfListen(False)
            self.loc, self.match = "inconnue", math.nan
            why = "carte %s gardee" % (self.map_name or "-")
        elif mode == "exploration":
            t0 = time.monotonic()
            ok, why = self._enterMapping(new=(arg == "new"))
            if not ok:
                return False, why
            # Nav2 ACTIF des l'entree, comme en navigation (demande utilisateur 2026-10-13) :
            # le premier ordre (A, X) attendait sinon 20 a 40 s son demarrage. Cout : ~0,5
            # coeur tant que NAV est allumee, y compris en cartographie manuelle.
            t1 = time.monotonic()
            ok2, why2 = self._nav2(True)
            if not ok2:
                why += " ; Nav2 " + why2
            why += " [slam %.0f s, Nav2 %.0f s]" % (t1 - t0, time.monotonic() - t1)
        else:
            if arg and arg != self.map_name:
                ok, why = self._switchMap(arg)
                if not ok:
                    return False, why
            t0 = time.monotonic()
            ok, why = self._enterLocalization()
            if not ok:
                return False, why
            t1 = time.monotonic()
            ok2, why2 = self._nav2(True)
            if not ok2:
                why += " ; Nav2 " + why2
            why += " [slam %.0f s, Nav2 %.0f s]" % (t1 - t0, time.monotonic() - t1)
        old, self.submode = self.submode, mode
        self._event("sous_mode", de=old, vers=mode)
        self.detail = "sous-mode %s : %s" % (mode, why)
        self.get_logger().info(self.detail)
        return True, self.detail

    def _switchMap(self, name):
        """Change de carte courante (cartographie / localisation arretees avant)."""
        if self.store.exists(name):
            pass
        elif name in map_io.list_maps(self.maps_dir):
            self.store.import_legacy(name)
        else:
            return False, "carte %s introuvable" % name
        if self.slamMode:
            self._saveMap("changement de carte", force=True)
            self._nav2(False)
            self._stopSlam()
        self._useMap(name)
        self._event("chargee")
        return True, "carte courante : %s" % name

    # ------------------------------------------------------------ buts
    def _robotPose(self, stamp=None):
        if self.tfl is None:
            return None
        t = None
        if stamp is not None:
            try:
                t = self.tf.lookup_transform("map", "base_footprint", Time.from_msg(stamp),
                                             timeout=Duration(seconds=0.2))
            except Exception:                               # noqa: BLE001
                t = None
        if t is None:
            try:
                t = self.tf.lookup_transform("map", "base_footprint", Time(),
                                             timeout=Duration(seconds=0.5))
            except Exception:                               # noqa: BLE001
                return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                map_io.yaw_of(q.z, q.w, q.x, q.y))

    def _publishPose(self):
        if self.tfl is None or not self.slamMode:
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

    def _setNavEnable(self, on):
        if on != self._navEnabled:
            self._navEnabled = on
            self.pubEnable.publish(Bool(data=on))

    def _goto(self, wp, kind="dest"):
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
        self._goalInfo = {"kind": kind, "wp": dict(wp)}
        self.distance, self.recoveries = math.nan, 0
        # Buts NUMEROTES : l'accuse et le resultat d'un but ANNULE peuvent arriver apres
        # l'envoi du suivant (exploration auto : pause manette, gate suivante) ; sans le
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
        kind = self._goalInfo.get("kind")
        if kind in ("dest", "home", "patrol"):
            self._event("destination", nom=self.goal_name, resultat=self.goal_status)
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
            try:
                ok, msg = self.command(req.command.strip(), req.arg.strip())
            except Exception as e:                          # noqa: BLE001
                ok, msg = False, "erreur interne : %s" % e
                self.get_logger().error("commande %s : %s" % (req.command, e))
        resp.success, resp.message = ok, msg
        self.detail = msg
        self._publish()
        return resp

    def command(self, cmd, arg):
        if cmd in ("exploration", "navigation", "idle"):
            # Le sous-mode DEMANDE (A long, croix) est retenu ; seul un arret explicite
            # l'oublie. Une carte forcee qui echoue retombe en idle, et la carte suivante
            # choisie au + / - relance ce sous-mode (essai du 2026-10-13 : NAV restait eteinte).
            self._wantSub = None if cmd == "idle" else cmd
            return self.setSubmode(cmd, arg)
        if cmd == "new_map":
            return self._newMap()
        if cmd == "load_map":
            return self._loadMap(arg)
        if cmd == "delete_map":
            return self._deleteMap(arg)
        if cmd == "auto_explore":
            if self.auto in ("running", "paused", "homing"):
                return self._autoStop("demande")
            return self._autoStart()
        if cmd == "explore_gate":
            return self._exploreGate(arg)
        if cmd == "save_map":
            if self.submode == "idle" or not self.slamMode:
                return False, "rien a sauver en idle (la carte %s est sur le disque)" % (
                    self.map_name or "-")
            return self._saveMap("demande", force=True)
        if cmd == "select":
            return self._select(arg)
        if cmd in ("dest_add", "save_waypoint"):
            return self._destAdd(arg)
        if cmd == "dest_del":
            return self._destDel(arg)
        if cmd == "dest_rename":
            key, _sp, new = arg.partition(" ")
            if not self.meta or not new.strip():
                return False, "usage : dest_rename <n|nom> <nouveau nom>"
            d = map_io.dest_rename(self.meta, key, new.strip())
            if d is None:
                return False, "destination %s inconnue" % key
            self._saveMeta()
            self._event("destination_renommee", n=d["n"], nom=d["name"])
            return True, "destination %d : %s" % (d["n"], d["name"])
        if cmd == "goto":
            return self._gotoCmd(arg)
        if cmd == "relocalize":
            if self.submode != "navigation":
                return False, "relocalisation : en navigation seulement"
            return self._relocStart(arg == "global", "demande")
        if cmd == "cancel":
            self._cancelGoal()
            return True, "but annule"
        if cmd == "cancel_all":
            return self._cancelAll()
        if cmd == "patrol":
            if self.patrol != "off":
                return self._patrolStop("demande")
            return self._patrolStart()
        if cmd == "list":
            maps = ["%s (%s)" % (m["name"], m["kind"]) for m in self.store.list()]
            dests = ["%d:%s" % (x["n"], x["name"]) for x in (self.meta or {}).get(
                "destinations", [])]
            gates = ["%d:%s" % (g["n"], g["status"]) for g in (self.meta or {}).get(
                "gates", []) if g["status"] != "exploree"]
            return True, "carte courante %s ; cartes : %s ; destinations : %s ; gates : %s" % (
                self.map_name or "-", ", ".join(maps) or "aucune", ", ".join(dests) or "aucune",
                ", ".join(gates) or "aucune")
        if cmd == "events":
            n = int(arg) if arg.isdigit() else 20
            return True, json.dumps(self.store.events(self.map_name, n) if self.map_name else [],
                                    ensure_ascii=False)
        return False, "commande inconnue : %s" % cmd

    def _newMap(self):
        """Carte VIERGE ; l'ancienne reste sur le disque. En NAV : cartographie aussitot."""
        if self.auto != "off":
            self._autoStop("nouvelle carte", nav_reset=False)
        self._cancelGoal()
        old = self.map_name
        if self.slamMode:
            self._saveMap("nouvelle carte", force=True)
        self._nav2(False)
        self._stopSlam()
        self._useMap(self.store.create())
        self._event("nouvelle_carte", precedente=old)
        if self.submode == "idle":
            return True, "nouvelle carte %s (vierge) ; %s gardee sur le disque" % (
                self.map_name, old or "-")
        self.submode = "idle"
        return self.setSubmode("exploration")

    def _deleteMap(self, name=""):
        """Carte COURANTE (ou nommee) a la CORBEILLE (demande utilisateur 2026-10-13, - long en
        exploration). Rien n'est sauve avant : c'est elle qu'on jette. Exploration et slam
        arretes, retour en idle ; la carte la plus recente qui reste devient la courante."""
        if not name and self._mapSel:
            # Une carte choisie au + / - n'est pas encore appliquee : - long jetterait
            # l'ANCIENNE carte courante, pas celle affichee.
            return False, "carte %s pas encore appliquee : attendre 2 s puis - long" % \
                self._mapSel
        name = name or self.map_name
        if not name or not self.store.exists(name):
            return False, "aucune carte a supprimer"
        if name == self.map_name:
            if self.auto != "off":
                self.auto, self._autoTarget, self._autoGate = "off", None, None
                self._autoOwnCancel = True
            self._cancelGoal()
            self._nav2(False)
            self._stopSlam()
            self._tfListen(False)
            self.submode, self.loc, self.match = "idle", "inconnue", math.nan
            self._mapSel = None
        self._event("supprimee")
        dst = self.store.trash(name)
        rest = [m["name"] for m in self.store.list() if m["kind"] == "carte"]
        if name == self.map_name:
            self._useMap(rest[0] if rest else "")
        self.get_logger().warn("carte %s mise a la corbeille (%s)" % (name, dst))
        return True, "carte %s a la corbeille ; carte courante : %s" % (
            name, self.map_name or "aucune (+ long : nouvelle carte)")

    def _loadMap(self, name):
        if not name:
            return False, "usage : load_map <carte> (cf. list)"
        # Sous-mode a reprendre : celui qui tourne, sinon celui que l'utilisateur a demande
        # (idle apres l'echec d'une carte forcee precedente).
        prev = self.submode if self.submode != "idle" else (self._wantSub or "idle")
        if self.submode != "idle":
            self.setSubmode("idle")
        ok, why = self._switchMap(name)
        if not ok or prev == "idle":
            return ok, why
        ok2, why2 = self.setSubmode(prev)
        return ok2, "%s ; %s" % (why, why2)

    def _selectMap(self, step):
        """+ / - de la manette : carte enregistree suivante / precedente (a graphe de poses, la
        plus recente d'abord), APPLIQUEE `map_select_delay_s` apres le dernier appui (on peut
        faire defiler sans relancer slam a chaque carte). Demande utilisateur 2026-10-13."""
        names = [m["name"] for m in self.store.list() if m["kind"] == "carte"]
        if not names:
            return False, "aucune carte enregistree"
        cur = self._mapSel or self.map_name
        k = names.index(cur) if cur in names else -1
        k = (k + step) % len(names) if k >= 0 else (0 if step > 0 else len(names) - 1)
        self._mapSel, self._mapSelT = names[k], time.monotonic()
        tag = " (courante)" if names[k] == self.map_name else ""
        return True, "carte %d/%d : %s%s -- appliquee dans %.0f s" % (
            k + 1, len(names), names[k], tag, self.map_select_delay_s)

    def _mapSelTick(self):
        """Applique la carte choisie au + / - : simple changement en idle ; sinon arret,
        changement, et reprise du sous-mode avec relocalisation sur CETTE carte seulement."""
        name = self._mapSel
        if not name or time.monotonic() - self._mapSelT < self.map_select_delay_s:
            return
        if not self._lock.acquire(blocking=False):
            return
        try:
            self._mapSel = None
            if name == self.map_name:
                self.detail = "carte %s gardee" % name
                return
            self._forcedMap = True
            ok, why = self._loadMap(name)
            self._forcedMap = False
            self._event("carte_forcee", carte=name, ok=ok)
            self.detail = ("carte forcee %s : %s" % (name, why)) if ok else \
                ("carte forcee %s : ECHEC, %s" % (name, why))
            self.get_logger().info(self.detail)
        finally:
            self._lock.release()

    def _select(self, arg):
        """gate+ gate- dest+ dest- | gate <n> | dest <n> | map+ map- : la selection de la
        manette."""
        a = arg.replace(" ", "")
        if a in ("map+", "map-"):
            return self._selectMap(+1 if a == "map+" else -1)
        kind = "gate" if a.startswith("gate") else "dest" if a.startswith("dest") else ""
        if not kind or self.meta is None:
            return False, "usage : select gate+|gate-|dest+|dest-|gate <n>|dest <n>"
        rest = a[4:]
        items = self._gateList() if kind == "gate" else self.meta["destinations"]
        # Le DEPART (numero 0) en tete des deux listes (demande utilisateur 2026-10-13) :
        # navigation, A y va ; exploration, X y retourne.
        if self._homePose() is not None:
            items = [{"n": 0}] + list(items)
        if rest in ("+", "-"):
            n = map_io.cycle(items, self.sel[kind], +1 if rest == "+" else -1)
        elif rest.isdigit() and any(int(i["n"]) == int(rest) for i in items):
            n = int(rest)
        else:
            return False, "%s %s inconnue" % (kind, rest)
        self.sel[kind] = n
        if n is None:
            return True, "aucune %s sur la carte %s" % (
                "gate" if kind == "gate" else "destination", self.map_name or "-")
        if n == 0:
            return True, "depart"
        if kind == "dest":
            d = map_io.dest_find(self.meta, n)
            return True, "destination %d : %s" % (n, d["name"])
        g = map_io.gate_find(self.meta, n)
        return True, "gate %d (%s)" % (n, g["status"])

    def _gateList(self):
        """Gates SELECTIONNABLES : celles qui restent a explorer (pas exploree)."""
        return [g for g in (self.meta or {}).get("gates", []) if g["status"] != "exploree"]

    def _destAdd(self, name):
        if self.meta is None:
            return False, "aucune carte courante"
        pose = self._robotPose()
        if pose is None:
            return False, "pose inconnue (TF map->base_footprint) : passer en NAV"
        d = map_io.dest_add(self.meta, pose[0], pose[1], pose[2], name)
        self.sel["dest"] = d["n"]
        self._saveMeta()
        self._event("destination_ajoutee", n=d["n"], nom=d["name"], pose=pose)
        return True, "destination %d %s enregistree (%.2f, %.2f)" % (d["n"], d["name"],
                                                                     pose[0], pose[1])

    def _destDel(self, key):
        if self.meta is None:
            return False, "aucune carte courante"
        if not key and self.sel["dest"] == 0:
            return False, "le depart ne se supprime pas"
        key = key or (str(self.sel["dest"]) if self.sel["dest"] else "")
        if not key:
            return False, "aucune destination selectionnee (L / R)"
        d = map_io.dest_del(self.meta, key)
        if d is None:
            return False, "destination %s inconnue" % key
        if self.sel["dest"] == d["n"]:
            self.sel["dest"] = map_io.cycle(self.meta["destinations"], None, +1)
        self._saveMeta()
        self._event("destination_supprimee", n=d["n"], nom=d["name"])
        return True, "destination %d %s supprimee" % (d["n"], d["name"])

    def _gotoCmd(self, arg):
        if self.submode not in ("navigation", "exploration"):
            return False, "passer d'abord en NAV (navigation ou exploration)"
        if self.auto in ("running", "paused", "homing"):
            return False, "exploration auto en cours (A pour l'arreter)"
        if self.loc in ("perdue", "relocalisation"):
            return False, "localisation %s : attendre (ou relocaliser, + long)" % self.loc
        meta = self.meta or {}
        if arg in ("", "selected") and self.sel["dest"] == 0:
            arg = "home"                       # « depart » choisi au L / R
        if arg == "home":
            home = self._homePose()
            if not home:
                return False, "depart inconnu sur la carte %s" % (self.map_name or "-")
            wp, kind = {"name": "depart", "x": home[0], "y": home[1], "yaw": home[2]}, "home"
        else:
            if arg in ("", "selected"):
                key = self.sel["dest"]
            elif arg == "next":
                key = map_io.cycle(meta.get("destinations", []), self.sel["dest"], +1)
            else:
                key = arg
            d = map_io.dest_find(meta, key) if key is not None and meta else None
            if d is None:
                return False, ("aucune destination selectionnee (L / R)" if not arg
                               else "destination %s inconnue" % arg)
            self.sel["dest"] = d["n"]
            wp, kind = d, "dest"
        ok, why = self._nav2(True)
        if not ok:
            return False, "Nav2 " + why
        return self._goto(wp, kind)

    def _homePose(self):
        """Depart de la carte (meta), sinon de la session d'exploration ; None si inconnu."""
        session = list(self._autoHome) if self._autoHome else None
        return (self.meta or {}).get("home") or session or None

    def _exploreGate(self, arg):
        if self.meta is None:
            return False, "aucune carte courante"
        if not arg and self.sel["gate"] == 0:
            # « depart » choisi au L / R : y retourner.
            if self.auto in ("running", "paused"):
                self._autoHoming("demande (depart choisi)")
                return True, "exploration auto : retour au depart"
            return self._gotoCmd("home")
        n = arg or (str(self.sel["gate"]) if self.sel["gate"] else "")
        g = map_io.gate_find(self.meta, n) if n else None
        if g is None or g["status"] == "exploree":
            return False, ("aucune gate selectionnee (L / R)" if not n
                           else "gate %s inconnue ou deja exploree" % n)
        self._autoPrio = int(g["n"])
        self._event("gate_designee", n=g["n"])
        if self.auto in ("running", "paused"):
            # La gate en cours cede la place : interrompue, la designee au tour suivant.
            if self._autoGate is not None and self._autoGate is not g:
                self._autoOwnCancel = True
                self._cancelGoal()
                self._gateSet(self._autoGate, "ouverte", "remplacee par la gate %d" % g["n"])
                self._autoGate, self._autoTarget = None, None
            return True, "gate %d designee : visee tout de suite" % g["n"]
        ok, why = self._autoStart()
        if ok:
            self._autoOnce = True          # X : cette gate, puis arret (2026-10-13)
        return ok, ("gate %d visee, arret a l'arrivee ; %s" % (g["n"], why)) if ok else why

    # ------------------------------------------------------------ exploration automatique
    # Strategie (choisie le 2026-10-11, gates le 2026-10-12) : aller de GATE en gate (limite
    # connu / inconnu, bamboo_navigation.frontier ; numerotee et suivie dans la meta de la
    # carte) jusqu'a ce qu'il n'en reste plus, puis REVENIR au depart, sauver la carte et
    # s'arreter. Le stick met en PAUSE (le multiplexeur lui donne deja la main) ;
    # l'exploration repart seule `auto_resume_s` apres le relachement. Une gate DESIGNEE
    # (explore_gate) passe avant les autres.
    def _autoState(self):
        """Pour les pastilles EXP-AUTO / NAV-AUTO du HUD (NavState.auto_state)."""
        if self.auto in ("running", "paused", "homing", "finishing"):
            if self.auto == "homing":
                return "homing"
            return "gate" if self._autoOnce else "exploration"
        if self.patrol != "off":
            return "patrol"
        if self.goal_status == "active":
            kind = self._goalInfo.get("kind")
            return "home" if kind == "home" else "goal" if kind == "dest" else "off"
        return "off"

    def _cancelAll(self):
        """B court de la manette en NAV (2026-10-13) : tout but annule, exploration auto et
        ronde arretees, le robot s'arrete."""
        what = []
        if self.auto in ("running", "paused", "homing"):
            self._autoStop("B (manette)")
            what.append("exploration auto")
        if self.patrol != "off":
            self._patrolStop("B (manette)")
            what.append("ronde")
        if self.goal_status == "active":
            what.append("but %s" % self.goal_name)
        self._cancelGoal()
        self._event("annulation", objets=what)
        return True, "arret : %s" % (", ".join(what) or "rien en cours")

    # ------------------------------------------------------------ ronde des destinations
    # A en navigation (demande utilisateur 2026-10-13) : le robot va de destination en
    # destination, EN BOUCLE, dans l'ordre le plus COURT (longueurs de trajet du
    # planificateur, pas a vol d'oiseau), jusqu'a un nouvel A ou B. Le stick met en pause,
    # comme l'exploration auto ; une destination en echec est sautee.
    PATROL_EXACT_MAX = 7                  # au-dela : plus proche voisin (7! = 5040 ordres)

    def _planLen(self, a, b):
        """Longueur (m) du trajet planifie de a a b (x, y), None si aucun chemin."""
        if not self.planner.wait_for_server(timeout_sec=1.0):
            return None
        g = ComputePathToPose.Goal()
        g.use_start = True
        for ps, pt in ((g.start, a), (g.goal, b)):
            ps.header.frame_id = "map"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = float(pt[0]), float(pt[1])
            ps.pose.orientation.w = 1.0
        fut = self.planner.send_goal_async(g)
        if not self._wait(fut, 3.0) or not fut.result().accepted:
            return None
        res = fut.result().get_result_async()
        if not self._wait(res, 5.0) or res.result().status != 4:
            return None
        pts = [(p.pose.position.x, p.pose.position.y) for p in res.result().result.path.poses]
        if len(pts) < 2:
            return math.hypot(b[0] - a[0], b[1] - a[1])
        return sum(math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])
                   for k in range(len(pts) - 1))

    def _patrolPlan(self, pose, ds):
        """Ordre (numeros) de la ronde : le CYCLE le plus court, entame par la destination la
        plus proche du robot. Longueurs du planificateur ; sans chemin, vol d'oiseau x 3."""
        import itertools
        pts = [(pose[0], pose[1])] + [(d["x"], d["y"]) for d in ds]
        n = len(pts)
        L = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                v = self._planLen(pts[i], pts[j])
                if v is None:
                    v = 3.0 * math.hypot(pts[j][0] - pts[i][0], pts[j][1] - pts[i][1])
                L[i][j] = L[j][i] = v
        idx = list(range(1, n))
        if len(idx) <= self.PATROL_EXACT_MAX:
            def cost(order):
                return sum(L[order[k]][order[(k + 1) % len(order)]] for k in range(len(order)))
            best = min(itertools.permutations(idx), key=cost)
            k0 = min(range(len(best)), key=lambda k: L[0][best[k]])   # entree la plus proche
            order = list(best[k0:]) + list(best[:k0])
        else:
            order, left, cur = [], set(idx), 0
            while left:
                cur = min(left, key=lambda j: L[cur][j])
                order.append(cur)
                left.remove(cur)
        return [ds[k - 1]["n"] for k in order]

    def _patrolStart(self):
        if self.submode != "navigation":
            return False, "ronde : en navigation seulement"
        if self.loc in ("perdue", "relocalisation"):
            return False, "ronde refusee : localisation %s" % self.loc
        ds = (self.meta or {}).get("destinations", [])
        if not ds:
            return False, "ronde : aucune destination (Y pour en creer)"
        pose = self._robotPose()
        if pose is None:
            return False, "ronde : pose inconnue"
        ok, why = self._nav2(True)
        if not ok:
            return False, "ronde : Nav2 " + why
        self._cancelGoal()
        self._patrolOrder = self._patrolPlan(pose, ds)
        self._patrolIdx, self._patrolSent = 0, False
        self.patrol = "running"
        names = [map_io.dest_find(self.meta, n)["name"] for n in self._patrolOrder]
        self._event("ronde", etat="debut", ordre=names)
        self.get_logger().info("ronde : %s" % " -> ".join(names))
        return True, "ronde lancee : %s (en boucle)" % " -> ".join(names)

    def _patrolStop(self, why):
        self.patrol = "off"
        if self._goalInfo.get("kind") == "patrol":
            self._cancelGoal()
        self._event("ronde", etat="arret", raison=why)
        self.detail = "ronde arretee (%s)" % why
        return True, self.detail

    def _patrolTick(self):
        if self.patrol == "off" or self._relocBusy:
            return
        left_nav = bool(self._robotMode) and self._robotMode != "navigation"
        if self.submode != "navigation" or left_nav:
            with self._lock:
                self._patrolStop("sortie de la navigation")
            return
        now = time.monotonic()
        stick = now - self._teleopT < self.auto_resume_s
        if stick or self._armed is False:
            if self.patrol == "running":
                with self._lock:
                    self.patrol = "paused"
                    if self._goalInfo.get("kind") == "patrol":
                        self._cancelGoal()
                    self._patrolSent = False       # la destination en cours sera reprise
            self.detail = "ronde : PAUSE (%s)" % (
                "manette" if stick else "traction desarmee : B long pour reprendre")
            return
        if self.patrol == "paused":
            self.patrol = "running"
        if self.goal_status == "active":
            return
        with self._lock:
            if self.patrol != "running" or not self._patrolOrder:
                return
            if self._patrolSent:               # destination precedente finie (ou en echec)
                self._patrolIdx = (self._patrolIdx + 1) % len(self._patrolOrder)
            d = None
            for _k in range(len(self._patrolOrder)):
                d = map_io.dest_find(self.meta, self._patrolOrder[self._patrolIdx])
                if d is not None:
                    break                      # (une destination supprimee est sautee)
                self._patrolIdx = (self._patrolIdx + 1) % len(self._patrolOrder)
            if d is None:
                self._patrolStop("plus aucune destination")
                return
            ok, why = self._goto(d, "patrol")
            self._patrolSent = ok
            self.detail = "ronde : vers %s (%d/%d)" % (d["name"], self._patrolIdx + 1,
                                                       len(self._patrolOrder))

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

    def _gyroRate(self):
        """Vitesse de lacet mesuree par le gyro (rad/s, dernier /imu/data), ou None."""
        raw = self._imuRaw
        if raw is None:
            return None
        return float(deserialize_message(raw, Imu).angular_velocity.z)

    def _waitStill(self, timeout=4.0, need=1.0):
        """Le robot est-il IMMOBILE selon les ROUES (odometrie brute des codeurs) pendant `need`
        s ? Attend jusqu'a `timeout` (un but vient d'etre annule : le robot s'arrete en moins
        d'une seconde). PAS selon l'EKF : il porte le biais du gyro -- essai G1c (2026-10-13),
        un biais de -0,043 rad/s laisse par une recalibration ratee faisait juger le robot « en
        mouvement », et refuser la recalibration qui l'aurait corrige."""
        t0, still0 = time.monotonic(), None
        while time.monotonic() - t0 < timeout:
            raw = self._wheelRaw if self._wheelRaw is not None else self._odomRaw
            if raw is not None:
                tw = deserialize_message(raw, Odometry).twist.twist
                if abs(tw.linear.x) < 0.02 and abs(tw.angular.z) < 0.05:
                    still0 = still0 or time.monotonic()
                    if time.monotonic() - still0 >= need:
                        return True
                else:
                    still0 = None
            time.sleep(0.1)
        return False

    def _autoStart(self):
        # Le robot doit etre IMMOBILE : la carte moyenne le gyro ~2 s, et un mouvement
        # pendant ce temps deviendrait un biais.
        if time.monotonic() - self._teleopT < self.auto_still_s:
            return False, ("exploration auto refusee : robot en mouvement (lacher le stick "
                           "%.0f s)" % self.auto_still_s)
        if self._relocBusy or self.loc in ("douteuse", "perdue", "relocalisation"):
            return False, ("exploration auto refusee : localisation %s (attendre, ou "
                           "nouvelle carte : + long)" % self.loc)
        if not math.isnan(self._battPct) and self._battPct < self.auto_battery_min_pct:
            # Sinon la session part et repart aussitot au depart (essai G1b : gate designee a
            # 15 %, retour immediat).
            return False, "exploration auto refusee : batterie %.0f %% (recharger)" % \
                self._battPct
        # Session RELANCEE en cours de route (gate designee pendant un retour au depart) :
        # on garde son DEPART. Essai G1b (2026-10-13) : le depart de la nouvelle session
        # etait pris la ou le robot se trouvait.
        prev_home = self._autoHome if self.auto in ("running", "paused", "homing") else None
        if self.auto in ("running", "paused", "homing"):
            self.auto = "off"              # la boucle n'envoie plus rien
            self._autoOwnCancel = True
            self._gateSet(self._autoGate, "ouverte", "session relancee")
            self._autoGate, self._autoTarget = None, None
        self._cancelGoal()
        # La carte COURANTE est completee, a la pose RETROUVEE du robot (relocalisation au
        # passage en exploration). Avant le 2026-10-12, chaque lancement repartait d'une carte
        # neuve : reprendre l'ancienne session sans relocaliser etait dangereux (robot deplace
        # a la main, essai E2 : « il part sur une ancienne carte »).
        if self.submode != "exploration":
            ok, why = self.setSubmode("exploration")
            if not ok:
                return False, "exploration auto impossible : " + why
        else:
            ok, why = self._enterMapping()
            if not ok:
                return False, "exploration auto impossible : " + why
        # Recalibration du gyro SEULEMENT robot IMMOBILE (odometrie, pas seulement le stick) et
        # pas plus d'une fois par `auto_recal_min_interval_s`. Essais G1 et G1b (2026-10-13) :
        # recalibre pendant que Nav2 ramenait le robot au depart, le gyro a pris la rotation
        # en cours pour un biais, la carte ESP32 est restee 2 s sans lire ses codeurs -> pose
        # du robot hors de la carte, carte dedoublee et tournee.
        since = time.monotonic() - self._lastRecalT
        still = self._waitStill()
        # Roues immobiles mais gyro qui « tourne » : biais derive (ou recalibration ratee),
        # recalibrer quel que soit l'intervalle.
        rate = self._gyroRate() if still else None
        drift = rate is not None and abs(rate) > self.auto_gyro_drift_rad_s
        if not still:
            recal_t, recal_msg = None, "gyro NON recalibre (roues en mouvement)"
            self.get_logger().warn("exploration auto : " + recal_msg)
        elif since < self.auto_recal_min_interval_s and not drift:
            recal_t, recal_msg = None, "gyro deja recalibre il y a %.0f s" % since
        else:
            if drift:
                self.get_logger().warn("exploration auto : gyro a %.3f rad/s roues immobiles "
                                       "-> recalibration forcee" % rate)
                self._event("gyro_derive", vitesse_rad_s=rate)
            recal_t, recal_msg = self._recalGyro()
            if recal_t is None:
                self.get_logger().warn("exploration auto : " + recal_msg)
            else:
                self._lastRecalT = recal_t
        # Le demarrage de Nav2 (~10 s) recouvre l'attente de la recalibration.
        ok, why = self._nav2(True)
        if not ok:
            return False, "exploration auto impossible : Nav2 " + why
        if recal_t is not None:
            rest = self.auto_recal_settle_s - (time.monotonic() - recal_t)
            if rest > 0:
                time.sleep(rest)
        self._autoOnce = False             # A : continu (X le repasse a vrai)
        if self.patrol != "off":
            self._patrolStop("exploration auto")
        self.auto, self._autoResumeTo = "running", "running"
        self._autoT0 = time.monotonic()
        self._autoHome, self._autoTarget, self._autoGate = prev_home, None, None
        self._autoNoFront, self._autoHomeTries, self._autoSecond = 0, 0, False
        # Nouvelle session : les gates en echec d'hier ont droit a un nouvel essai.
        map_io.gates_reopen(self.meta)
        self._saveMeta()
        self._event("exploration_auto", etat="debut", gyro=recal_msg)
        self.get_logger().info("exploration auto : demarrage sur %s, %s"
                               % (self.map_name, recal_msg))
        return True, "exploration auto lancee sur %s, %s" % (self.map_name, recal_msg)

    def _autoStop(self, why, nav_reset=True):
        # L'etat bascule EN PREMIER : la boucle peut etre au milieu d'un tour (carte demandee
        # hors verrou) ; elle revoit `auto` sous le verrou avant tout envoi. Essai E1 : sans
        # cela, une gate est partie 2 s APRES l'arret.
        self.auto, self._autoTarget = "off", None
        self._autoOwnCancel = True
        self._cancelGoal()
        if self._autoGate is not None:
            self._gateSet(self._autoGate, "ouverte", "exploration arretee")
            self._autoGate = None
        self._autoPrio = None
        self._autoOnce = False
        self._event("exploration_auto", etat="arret", raison=why)
        # TOUT arret sauve la carte (essai E2d : arret sur "retour au depart impossible", puis
        # relance -> carte de la 1re exploration PERDUE).
        self._saveMap("arret de l'exploration auto (%s)" % why, force=True)
        # Nav2 reste ACTIF : il est demarre avec le sous-mode et arrete avec lui (idle).
        self.detail = "exploration auto arretee (%s)" % why
        self.get_logger().info(self.detail)
        return True, self.detail

    def _autoGoto(self, name, x, y, yaw, kind="gate"):
        self._autoTarget, self._autoGoalT, self._autoOwnCancel = (x, y), time.monotonic(), False
        self._autoProgT, self._autoBestD = self._autoGoalT, math.inf
        return self._goto({"name": name, "x": x, "y": y, "yaw": yaw}, kind)

    def _gateSet(self, g, status, why=""):
        """Statut d'une gate : meta sauvee et evenement a chaque changement."""
        if g is None or g["status"] == status:
            return
        g["status"] = status
        self._saveMeta()
        self._event("gate", n=g["n"], statut=status, raison=why, x=g["x"], y=g["y"])

    def _autoMap(self):
        """Carte EN COURS de slam_toolbox, demandee hors du verrou (appel de service)."""
        if not self.mapCli.service_is_ready():
            return None
        fut = self.mapCli.call_async(GetMap.Request())
        if not self._wait(fut, 5.0):
            return None
        return fut.result().map

    def _autoTick(self):
        # UN SEUL tour a la fois : un tour (carte + verification des chemins) depasse parfois
        # 1 s, et l'executeur multi-fils lancait alors le suivant EN PARALLELE -- deux gates
        # envoyees a 0,7 s d'intervalle, la premiere restee `en_cours` (essai G1d, 2026-10-13).
        if not self._autoTickLock.acquire(blocking=False):
            return
        try:
            self._autoTickOnce()
        finally:
            self._autoTickLock.release()

    def _autoTickOnce(self):
        if self.auto in ("off", "done", "finishing"):
            return
        if self._relocBusy:
            # Retour en arriere / relocalisation en cours : slam_toolbox redemarre, rien a
            # envoyer ; l'exploration reprend apres.
            self.detail = "exploration auto : PAUSE (relocalisation en cours)"
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
                    self._gateSet(self._autoGate, "ouverte", "interrompue (manette)")
                    self._autoGate = None
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
            if self.meta is not None and not self.meta.get("home"):
                self.meta["home"] = [round(v, 3) for v in pose]
                self._saveMeta()
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
                    if self._autoGate is not None:
                        self._autoGate["tries"] = int(self._autoGate.get("tries", 0)) + 1
                    self._gateSet(self._autoGate, "abandonnee", "sans progres")
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
            elif not self._autoOwnCancel and self._autoGate is not None:
                g = self._autoGate
                if ok:
                    self._gateSet(g, "atteinte")
                    self._saveMap("gate %d atteinte" % g["n"])
                elif g["status"] == "en_cours":
                    g["tries"] = int(g.get("tries", 0)) + 1
                    self._gateSet(g, "abandonnee", "Nav2 : %s" % self.goal_status)
                    self.get_logger().info("exploration auto : gate %d %s" % (
                        g["n"], self.goal_status))
                if self._autoPrio == g["n"]:
                    self._autoPrio = None
                if self._autoOnce:
                    # X : la gate visee est traitee (atteinte ou en echec) -> arret.
                    with self._lock:
                        self._autoStop("gate %d %s" % (g["n"], g["status"]))
                    return
            self._autoTarget = None
            self._autoGate = None
        if self.auto == "homing":
            self._autoGotoHome()
            return
        # --- gate suivante ---
        m = self._autoMap()
        if m is None:
            self.detail = "exploration auto : carte indisponible (slam_toolbox ?)"
            return
        grid = frontier.as_grid(m.data, m.info.width, m.info.height)
        fs = frontier.find_frontiers(grid, m.info.resolution,
                                     (m.info.origin.position.x, m.info.origin.position.y),
                                     min_size_m=self.auto_min_frontier_m)
        self._autoKnown, _occ = map_io.map_stats(m.info.width, m.info.height, m.data)
        with self._lock:
            if self.auto != "running" or self.meta is None:
                return
            live = map_io.gates_update(self.meta, fs)
            sig = tuple((g["n"], g["status"]) for g in self.meta["gates"])
            if sig != self._gatesSig:
                self._gatesSig = sig
                self._saveMeta()
        self._autoFronts = len(live)
        self._publishMarks()
        prio = None
        if self._autoPrio is not None:
            prio = next((g for g in live if g["n"] == self._autoPrio), None)
            if prio is None:
                self._event("gate", n=self._autoPrio, statut="exploree",
                            raison="designee, deja connue")
                self._autoPrio = None
        if self._autoOnce and prio is None:
            with self._lock:
                self._autoStop("gate visee deja connue")
            return
        # On ne vise qu'une gate ATTEIGNABLE : demander d'abord un chemin au planificateur
        # (< 1 s). Essai E2 (2026-10-11) : 4 frontieres sur 6 etaient sous des meubles (le
        # lidar, a 17 cm, voit sous un canape ; le robot n'y passe pas) ; Nav2 y perdait ~30 s
        # chacune en manoeuvres de reprise avant d'abandonner.
        # X (une seule gate) : aucune autre candidate.
        cands = [] if self._autoOnce else [g for g in live if g["status"] == "ouverte"]
        f, bad = None, []
        for _try in range(4):
            if prio is not None and prio not in bad:
                cand = prio
            else:
                rest = [g for g in cands if g not in bad]
                cand = frontier.choose(rest, pose[:2])
            if cand is None:
                break
            ok_plan = self._planOk(cand["x"], cand["y"])
            if ok_plan is not False:           # chemin trouve, ou verification impossible
                f = cand
                break
            bad.append(cand)
            if self.auto != "running":
                return
        # ECHECS EN SERIE : 3 gates d'affilee sans chemin, c'est presque toujours le ROBOT
        # qui est dans une zone que la costmap croit occupee (arrete au ras d'un obstacle), pas
        # trois gates inaccessibles -- essai E2d : 11 condamnees en 2 s. On vide les costmaps
        # et on reessaie au tour suivant AVANT de condamner quoi que ce soit.
        if len(bad) >= 3 and time.monotonic() - self._autoClearT > 60.0:
            self._clearCostmaps("%d gates sans chemin d'affilee" % len(bad))
            return
        prio_failed = prio is not None and prio in bad
        if prio_failed:
            # Le DIRE, meme si l'exploration continue avec une autre gate (essai G2 : la gate 14
            # designee, deja inatteignable, a ete ignoree sans un mot).
            self._event("gate_designee_refusee", n=prio["n"], raison="aucun chemin")
            self.detail = "exploration auto : gate %d designee : aucun chemin" % prio["n"]
            self.get_logger().warn(self.detail)
        for g in bad:
            self._gateSet(g, "inatteignable", "aucun chemin")
            if g is prio:
                self._autoPrio = None
            self.get_logger().info("exploration auto : gate %d (%.2f, %.2f) INATTEIGNABLE"
                                   % (g["n"], g["x"], g["y"]))
        if f is None:
            if any(g["status"] == "ouverte" for g in live):
                return                         # d'autres candidates : au tour suivant
            if prio_failed:
                # La gate DESIGNEE n'a pas de chemin et il n'y a rien d'autre : le dire et
                # RESTER sur place, plutot que de rentrer au depart (essai G1 : quatre
                # designations de suite, quatre retours au depart pour rien).
                with self._lock:
                    self._autoStop("gate %d designee inatteignable" % prio["n"])
                return
            retry = [g for g in live if g["status"] == "abandonnee" and
                     int(g.get("tries", 0)) < 2]
            if not self._autoSecond and retry:
                # SECONDE CHANCE, une fois : une gate abandonnee (robot coince contre un
                # chambranle, essai E2b : la porte du haut) redevient souvent atteignable avec
                # la carte agrandie et depuis une autre position. Les INATTEIGNABLES (aucun
                # chemin) ne sont pas retentees dans la session.
                self._autoSecond = True
                for g in retry:
                    self._gateSet(g, "ouverte", "seconde chance")
                self.get_logger().info("exploration auto : seconde chance pour %d gates "
                                       "abandonnees" % len(retry))
                return
            self._autoNoFront += 1
            reason = "carte complete" if not live else \
                "plus de gate atteignable (%d en echec)" % len(live)
            self.detail = "exploration auto : %s (%d/2)" % (reason, self._autoNoFront)
            if self._autoNoFront >= 2:
                self._autoHoming(reason)
            return
        self._autoNoFront = 0
        yaw = math.atan2(f["y"] - pose[1], f["x"] - pose[0])
        with self._lock:
            if self.auto != "running":     # arrete ou mis en pause pendant ce tour
                return
            ok, why = self._autoGoto("gate %d" % f["n"], f["x"], f["y"], yaw)
            if not ok:                     # Nav2 absent : rien n'est parti, on le dit
                self._autoTarget = None
                self.detail = "exploration auto : " + why
                return
            self._autoGate = f
            self._gateSet(f, "en_cours", "designee" if f is prio else "")
            self.detail = "exploration auto : gate %d, %d ouvertes, %.0f %% connue" % (
                f["n"], len(cands), 100 * self._autoKnown)
        self.get_logger().info("exploration auto : gate %d -> (%.2f, %.2f), %.2f m de large, "
                               "%d en vue" % (f["n"], f["x"], f["y"], f.get("size_m", 0.0),
                                              len(live)))

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
            # La gate coupee par le retour redevient OUVERTE (a finir une autre fois).
            self._gateSet(self._autoGate, "ouverte", "retour au depart")
            self._autoGate = None
            self._autoOwnCancel = True
            self._cancelGoal()
            self.auto, self._autoResumeTo, self._autoTarget = "homing", "homing", None
            self._event("exploration_auto", etat="retour", raison=why)
            self.detail = "exploration auto : retour au depart (%s)" % why

    def _autoGotoHome(self):
        if self._autoHome is None:
            with self._lock:
                self._autoStop("depart inconnu")
            return
        x, y, yaw = self._autoHome
        # Pas de chemin JUSQU'AU DEPART : inutile d'user 3 fois 25 s de reprises Nav2 (essai G2,
        # 2026-10-13 : 42 plans refuses depuis la piece du bas, porte etroite pour le
        # planificateur alors que le robot y etait passe a l'aller). Un vidage des costmaps,
        # un nouvel essai au tour suivant, puis arret sur place avec la raison.
        if self._planOk(x, y) is False:
            if time.monotonic() - self._autoClearT > 30.0:
                self._clearCostmaps("aucun chemin vers le depart")
                return
            with self._lock:
                self._autoStop("aucun chemin vers le depart (passage trop etroit pour le "
                               "planificateur ?) : ramener le robot au stick")
            return
        with self._lock:
            if self.auto != "homing":
                return
            ok, why = self._autoGoto("retour depart", x, y, yaw, "homing")
            if not ok:
                self._autoTarget = None
                self.detail = "exploration auto : retour impossible, " + why

    def _autoFinish(self):
        # Une seule fois : l'arret de Nav2 qui suit dure ~30 s, et la boucle, qui avait vu le
        # retour reussi, rappelait cette fin a chaque tour (carte sauvee 3 fois, essai E2).
        if self.auto != "homing":
            return
        self.auto = "finishing"
        _ok, msg = self._saveMap("exploration terminee", force=True)
        self.auto, self._autoTarget = "done", None
        self._event("exploration_auto", etat="terminee",
                    duree_s=round(time.monotonic() - self._autoT0))
        self.detail = "exploration auto TERMINEE en %.0f s : %s" % (
            time.monotonic() - self._autoT0, msg)
        self.get_logger().info(self.detail)

    # ------------------------------------------------------------ etat et diagnostic
    def _publishMarks(self):
        """Depart et gates du moment pour la mini-carte (sujets d'avant le 2026-10-13, lus par
        l'overlay) ; le detail complet part sur /navigation/objects (JSON)."""
        h = PoseStamped()
        home = self._autoHome if self.auto != "off" and self._autoHome is not None else (
            (self.meta or {}).get("home"))
        if home:
            h.header.frame_id = "map"
            h.pose.position.x, h.pose.position.y = float(home[0]), float(home[1])
        self.pubHome.publish(h)
        a = PoseArray()
        a.header.frame_id = "map"
        for g in self._gateList():
            if g["status"] in ("ouverte", "en_cours"):
                p = Pose()
                p.position.x, p.position.y = float(g["x"]), float(g["y"])
                a.poses.append(p)
        self.pubFronts.publish(a)

    def _publishObjects(self):
        meta = self.meta or {}
        doc = {"map": self.map_name, "home": meta.get("home"),
               "gates": [{"n": g["n"], "x": g["x"], "y": g["y"], "status": g["status"]}
                         for g in meta.get("gates", [])],
               "destinations": [{"n": x["n"], "name": x["name"], "x": x["x"], "y": x["y"],
                                 "yaw": x["yaw"]} for x in meta.get("destinations", [])],
               "selected": dict(self.sel), "loc": self.loc,
               "match": None if math.isnan(self.match) else round(self.match, 3)}
        sig = json.dumps(doc, sort_keys=True)
        if sig == self._objSig:
            return
        self._objSig = sig
        self.pubObjs.publish(String(data=sig))
        # Ancien format du debriefing (outil MCP `map`) : numero, position, statut.
        self.pubLog.publish(String(data=json.dumps({
            "map": self.map_name, "home": (meta.get("home") or [None, None])[:2]
            if meta.get("home") else None,
            "frontiers": [{"n": g["n"], "x": g["x"], "y": g["y"], "s": g["status"]}
                          for g in meta.get("gates", [])]})))
        self._publishMarks()

    def _publishPreview(self):
        """Apercu de la carte choisie (+ / -) ou, slam arrete, de la carte courante ; une
        grille vide l'efface. Republie seulement quand la carte a montrer change."""
        want = self._mapSel or ("" if self.slamMode else self.map_name)
        if want == self._previewName:
            return
        self._previewName = want
        g = OccupancyGrid()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        if want and self.store.has_image(want):
            try:
                w, h, res, origin, data = map_io.read_map(self.store.path(want, "map.yaml"))
            except (OSError, ValueError, KeyError) as e:
                self.get_logger().warn("apercu %s illisible : %s" % (want, e))
            else:
                g.info.width, g.info.height, g.info.resolution = w, h, float(res)
                g.info.origin.position.x, g.info.origin.position.y = origin[0], origin[1]
                g.info.origin.orientation.w = 1.0
                g.data = data
        self.pubPreview.publish(g)

    def _watchModeExit(self):
        """Sortie du mode NAV : avec la navigation A LA DEMANDE, ce conteneur sera arrete 2 min
        plus tard, et slam_toolbox avec lui -- on sauve donc la carte DES la sortie, une fois."""
        mode = self._robotMode
        if mode and mode != "navigation" and self._lastRobotMode == "navigation":
            with self._lock:
                if self.slamMode:
                    self._saveMap("sortie du mode NAV", force=True)
        if mode:
            self._lastRobotMode = mode

    def _publish(self):
        self._watchModeExit()
        meta = self.meta or {}
        m = NavState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.submode, m.map_name = self.submode, self.map_name
        m.goal, m.goal_status = self.goal_name, self.goal_status
        m.distance_remaining = float(self.distance)
        m.recoveries = int(self.recoveries)
        m.localization_cov = math.nan
        m.waypoints = [x["name"] for x in meta.get("destinations", [])]
        m.missing_sensors = self.missing()
        m.ready = not m.missing_sensors
        m.detail = self.detail
        if hasattr(m, "gate_n"):
            # Champs du 2026-10-13 : absents tant que bamboo_interfaces n'est pas recompile.
            m.loc_state = self.loc
            m.map_match = float(self.match)
            kind = "gate" if self.submode == "exploration" else \
                "dest" if self.submode == "navigation" else ""
            n = self.sel.get(kind) if kind else None
            # n == 0 : le DEPART (selected_name "depart") ; None : rien de choisi.
            m.selected_kind, m.selected_n = (kind if n is not None else ""), int(n or 0)
            d = map_io.dest_find(meta, n) if kind == "dest" and n else None
            m.selected_name = d["name"] if d else ("depart" if n == 0 else "")
            if self._mapSel:
                # Carte en cours de choix (+ / -) : affichee a la place de la selection.
                m.selected_kind, m.selected_n, m.selected_name = "map", 1, self._mapSel
            gs = [g for g in meta.get("gates", []) if g["status"] != "exploree"]
            m.gate_n = [int(g["n"]) for g in gs]
            m.gate_x = [float(g["x"]) for g in gs]
            m.gate_y = [float(g["y"]) for g in gs]
            m.gate_status = [g["status"] for g in gs]
            ds = meta.get("destinations", [])
            m.dest_n = [int(x["n"]) for x in ds]
            m.dest_name = [x["name"] for x in ds]
            m.dest_x = [float(x["x"]) for x in ds]
            m.dest_y = [float(x["y"]) for x in ds]
        if hasattr(m, "auto_state"):
            m.auto_state = self._autoState()
        self.pubState.publish(m)
        self._publishObjects()
        self._publishPreview()

    def _diagNav(self, stat):
        miss = self.missing()
        stat.add("sous_mode", self.submode)
        stat.add("carte", self.map_name or "-")
        stat.add("slam", self.slamMode or "arrete")
        stat.add("localisation", self.loc)
        stat.add("accord_carte", "-" if math.isnan(self.match) else "%.2f" % self.match)
        stat.add("capteurs_exiges", ", ".join(self.required) or "aucun")
        stat.add("manquants", ", ".join(miss) or "aucun")
        stat.add("but", "%s %s" % (self.goal_name or "-", self.goal_status))
        stat.add("auto", self.auto)
        meta = self.meta or {}
        stat.add("destinations", str(len(meta.get("destinations", []))))
        stat.add("gates_ouvertes", str(sum(1 for g in meta.get("gates", [])
                                           if g["status"] == "ouverte")))
        if self.auto != "off":
            stat.add("gates_en_vue", str(self._autoFronts))
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
        elif self.submode == "navigation" and self.loc in ("perdue", "relocalisation"):
            stat.summary(DiagnosticStatus.WARN, "localisation %s" % self.loc)
        elif self.submode == "navigation" and self.loc == "douteuse":
            stat.summary(DiagnosticStatus.WARN, "carte modifiee ? (accord %.2f)" % self.match)
        else:
            stat.summary(DiagnosticStatus.OK, "pret (%s%s)" % (
                self.submode, (", " + self.map_name) if self.map_name else ""))
        return stat

    def shutdown(self):
        # PAS de sauvegarde ici : l'executeur ne tourne plus, les appels de service (GetMap,
        # SerializePoseGraph) n'auraient pas de reponse. La sauvegarde periodique et celles sur
        # evenement (dont la sortie du mode NAV, 2 min avant l'arret a la demande) y pourvoient.
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
