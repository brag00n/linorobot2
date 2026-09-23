#!/usr/bin/env python3
r"""esp32_mavlink_driver - pont MAVLink ESP32 <-> ROS2 (proprietaire UNIQUE du serie).

Reutilise TEL QUEL la couche hote `robot_control` : Esp32ComSerial (sous-classe de
RobotComSerial) ouvre le port CP210x en tache de fond, decode le protocole MAVLink
(dialecte bamboo, sysid=2) et expose snapshot()/encSpeed(). Aucun protocole reecrit ici.

Le noeud publie la telemetrie : odom integree (encodeurs), imu, batterie, rpm par roue,
magneto, et un etat brut de diagnostic (Esp32Status). L'ENVOI de /cmd_vel est GATE par le
parametre `enable_cmd_vel` :
  - false (defaut, lot L5) : LECTURE SEULE, aucune commande moteur emise ;
  - true  (lot L6) : actuation, apres calibration geometrie + roues surelevees.

Odometrie : la carte ne calcule PAS la pose (x, y, theta) ; on l'integre ici a partir des
2 voies encodeur (gauche/droite).

Geometrie et motorisation : ce noeud n'en declare AUCUN defaut. Elles viennent de la source
unique `config/robots/<robot>.yaml`, chargee par driver.launch.py. Un parametre physique
absent est une ERREUR FATALE et non un repli silencieux : les defauts litteraux dupliques
sont exactement ce qui a produit quatre diametres de roue contradictoires dans le depot.
Ces valeurs ne sont pas encore mesurees (lot 4) -> l'echelle de l'odom reste fausse.

RECONFIGURATION A CHAUD (lot 2) : ce noeud est le POINT DE JONCTION entre la
configuration ROS et la SRAM de la carte. Tout parametre physique peut etre change en
marche par `ros2 param set` (donc aussi depuis le MCP), et le changement est POUSSE
vers la carte puis RELU pour verification -- la relecture est publiee dans Esp32Status,
seule preuve que la carte a reellement pris la valeur. Meme `enable_cmd_vel` est
commutable a chaud : c'est ce qui rend l'actuation pilotable sans redemarrer le noeud,
l'orchestrateur MCP ne sachant pas passer d'argument de launch.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState, MagneticField
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

from bamboo_interfaces.msg import WheelRpm, Esp32Status

from robot_control.communication.Esp32ComSerial import Esp32ComSerial


def _quat_from_euler(roll, pitch, yaw):
    """Quaternion (x, y, z, w) depuis roll/pitch/yaw en RADIANS (convention ZYX)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Esp32MavlinkDriver(Node):
    # Parametres physiques attendus de la source unique config/robots/<robot>.yaml.
    # Aucun n'a de defaut : voir _req().
    _PHYS_DOUBLES = ("car_type", "counts_per_rev", "wheel_diameter_m", "wheel_separation_m",
                     "motor_max_rpm", "motor_operating_voltage", "motor_power_max_voltage",
                     "pid_kp", "pid_ki", "pid_kd")
    _PHYS_INTS = ("encoder_left_index", "encoder_right_index")
    _PHYS_BOOLS = ("invert_left", "invert_right")

    def _req(self, name):
        """Lit un parametre physique OBLIGATOIRE ; leve si la source unique ne l'a pas fourni.

        Selon la version de rclpy, un parametre declare sans valeur leve a la lecture ou
        renvoie une valeur None : les deux cas menent au meme echec explicite.
        """
        try:
            value = self.get_parameter(name).value
        except Exception:
            value = None
        if value is None:
            raise RuntimeError(
                f"parametre physique obligatoire absent : '{name}'. Il doit venir du fichier "
                "canonique bamboo_base/config/robots/<robot>.yaml (charge par "
                "driver.launch.py) ; ce noeud ne porte volontairement aucun defaut geometrique.")
        return value

    def __init__(self):
        super().__init__("esp32_mavlink_driver")

        # --- parametres de transport et de frames (faits d'hote : un defaut est licite) ---
        # read_only : ces trois-la sont STRUCTURELLEMENT figes une fois le noeud demarre --
        # le port est ouvert par un thread lecteur au constructeur, et la periode du timer
        # est capturee a sa creation. Les declarer read_only fait REJETER un `ros2 param set`
        # avec un message, au lieu de l'accepter et de ne rien changer : un reglage qui
        # semble pris mais reste sans effet coute plus cher a diagnostiquer qu'un refus.
        # Pour en changer : relancer le noeud (docker compose up -d driver.real).
        _fixed = ParameterDescriptor(
            read_only=True,
            description="fige au demarrage : relancer le noeud pour en changer")
        self.declare_parameter("port", "/dev/esp32", _fixed)
        self.declare_parameter("baud", 921600, _fixed)
        self.declare_parameter("publish_rate_hz", 30.0, _fixed)
        # --- parametres PHYSIQUES : declares SANS DEFAUT ---
        # Ils appartiennent a la source unique config/robots/<robot>.yaml. Declarer un type
        # sans valeur rend l'absence detectable : _req() leve alors une erreur nommant le
        # parametre, au lieu de laisser tourner l'odometrie sur un placeholder muet.
        for _name in self._PHYS_DOUBLES:
            self.declare_parameter(_name, Parameter.Type.DOUBLE)
        # encSpeed() renvoie [M1..M4] ; 2 voies physiques seulement (M3=M1, M4=M2 cote
        # encodeur). On mappe la voie gauche/droite par index.
        for _name in self._PHYS_INTS:
            self.declare_parameter(_name, Parameter.Type.INTEGER)
        for _name in self._PHYS_BOOLS:
            self.declare_parameter(_name, Parameter.Type.BOOL)
        self.declare_parameter("robot_base", Parameter.Type.STRING)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        # L'EKF possede le TF odom->base_footprint -> le driver ne le publie PAS par defaut.
        self.declare_parameter("publish_odom_tf", False)
        # Verrou d'actuation : false = lecture seule (L5), true = actuation (L6).
        # Commutable A CHAUD (cf. _apply_actuation) : c'est le canal d'armement depuis MCP.
        self.declare_parameter("enable_cmd_vel", False)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)

        # --- fiabilite QoS des capteurs ---------------------------------------
        # `sensor_data` (defaut ROS pour un capteur) est BEST-EFFORT, donc INVISIBLE a
        # travers rosbridge, qui souscrit en RELIABLE : l'IMU n'etait pas diagnosticable
        # par le MCP. `reliable` la rend lisible, au prix d'un peu de retransmission --
        # acceptable sur une IMU a 30 Hz en local, a ne pas generaliser a un lidar.
        self.declare_parameter("imu_qos", "sensor_data")

        # --- journal de la carte ----------------------------------------------
        # Seuil ecrit DANS LA CARTE (PARAM LOG_LEVEL, idx 19), pas un filtre local : le
        # firmware ne transmet pas ce qu'il filtre, ce qui menage l'UART partage avec la
        # telemetrie. MAV_SEVERITY : 0 EMERGENCY .. 3 ERROR, 4 WARNING, 6 INFO, 7 DEBUG.
        self.declare_parameter("board_log_level", 6)
        # Delai d'attente d'une RELECTURE de configuration. La relecture est bloquante (une
        # requete puis l'attente du rapport) : elle suspend donc brievement la publication.
        # Assume, car une reconfiguration est un acte rare et delibere, et la preuve que la
        # carte a pris la valeur vaut cette pause. La carte repond d'ordinaire en ~50 ms.
        self.declare_parameter("board_readback_timeout_s", 1.0)

        g = lambda n: self.get_parameter(n).value
        self.port = g("port")
        self.baud = int(g("baud"))
        rate = float(g("publish_rate_hz"))
        self.robot_base = str(self._req("robot_base"))
        self.car_type = float(self._req("car_type"))
        self.cpr = float(self._req("counts_per_rev"))
        self.wheel_diameter = float(self._req("wheel_diameter_m"))
        self.wheel_sep = float(self._req("wheel_separation_m"))
        # Motorisation et gains : lus ici pour echouer tot si la source est incomplete ;
        # leur diffusion vers la SRAM de la carte est le sujet du lot 2.
        self.motor_max_rpm = float(self._req("motor_max_rpm"))
        self.motor_voltage = float(self._req("motor_operating_voltage"))
        self.motor_max_voltage = float(self._req("motor_power_max_voltage"))
        self.pid = (float(self._req("pid_kp")), float(self._req("pid_ki")),
                    float(self._req("pid_kd")))
        self.left_idx = int(self._req("encoder_left_index"))
        self.right_idx = int(self._req("encoder_right_index"))
        self.inv_left = bool(self._req("invert_left"))
        self.inv_right = bool(self._req("invert_right"))
        self.odom_frame = g("odom_frame")
        self.base_frame = g("base_frame")
        self.imu_frame = g("imu_frame")
        self.publish_odom_tf = bool(g("publish_odom_tf"))
        self.enable_cmd_vel = bool(g("enable_cmd_vel"))
        self.cmd_vel_timeout = float(g("cmd_vel_timeout_s"))
        self.imu_qos_name = str(g("imu_qos"))
        self.board_log_level = int(g("board_log_level"))
        self.readback_timeout = float(g("board_readback_timeout_s"))
        # Derniere relecture reussie de la SRAM de la carte, publiee dans Esp32Status.
        self.board_cfg = None

        # --- lien serie (proprietaire unique) ---
        # vid_pid CP210x pour prioriser le bon port parmi le DOUBLE CP2102 (lidar + ESP32) ;
        # le sniff protocole + le sysid MAVLink departagent lidar vs ESP32 (cf. fiche materiel).
        self.link = Esp32ComSerial(self.port, self.baud, protocol="mavlink",
                                   cpr=self.cpr, vid_pid=["10C4:EA60"], telemetry=None)

        # --- publishers ---
        self.pub_odom = self.create_publisher(Odometry, "odom/unfiltered", 20)
        # QoS resolue une fois : un publisher ne change pas de fiabilite en marche (il
        # faudrait le detruire et le recreer, ce qui coupe les souscripteurs en place).
        # D'ou l'absence de `imu_qos` parmi les parametres a chaud, et sa presence parmi
        # les read_only serait trompeuse : il est lu au demarrage, point.
        _imu_qos = 10 if self.imu_qos_name == "reliable" else qos_profile_sensor_data
        self.pub_imu = self.create_publisher(Imu, "imu/data", _imu_qos)
        self.pub_batt = self.create_publisher(BatteryState, "battery", 10)
        self.pub_rpm = self.create_publisher(WheelRpm, "wheel_rpm", 10)
        self.pub_mag = self.create_publisher(MagneticField, "mag", _imu_qos)
        self.pub_status = self.create_publisher(Esp32Status, "esp32_status", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None

        # --- souscription cmd_vel (GATE par enable_cmd_vel, commutable a chaud) ---
        self._last_cmd_t = None
        self.sub_cmd = None
        self._apply_actuation(self.enable_cmd_vel)

        # --- etat odometrie integree ---
        self.x = self.y = self.th = 0.0
        self._last_t = self.get_clock().now()

        # --- diffusion de la configuration vers la SRAM de la carte ---
        # A L'INIT, et non seulement sur changement : la carte peut avoir redemarre seule
        # (elle n'a AUCUNE persistance, cf. lot 1.8) et serait alors revenue a ses valeurs
        # d'amorcage compilees. Le fichier canonique fait foi -> on le repousse a chaque
        # connexion, puis on relit pour le prouver.
        self._push_board_config(geom=True, pid=True, log_level=True)

        # Le journal de la carte est drainé a 2 Hz, pas dans _tick : ces lignes sont rares
        # et la file hote est bornee (200), donc rien ne presse -- inutile de payer ce
        # parcours 30 fois par seconde.
        self.log_timer = self.create_timer(0.5, self._drain_board_logs)

        # Enregistre EN DERNIER : le callback touche a des membres et au lien serie, il ne
        # doit donc pas pouvoir s'executer sur un objet a demi construit.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"driver ESP32 MAVLink demarre (port={self.port}, baud={self.baud}, cpr={self.cpr}).")

    # --- diffusion vers la carte -------------------------------------------
    def _push_board_config(self, geom=False, pid=False, log_level=False):
        """Ecrit la configuration dans la SRAM de la carte, puis la RELIT pour verification.

        `save=False` partout, et c'est delibere : l'ESP32 n'a AUCUNE persistance (il repond
        UNSUPPORTED a PREFLIGHT_STORAGE), donc demander la sauvegarde n'ajouterait qu'une
        trame inutile sur l'UART. C'est aussi pourquoi la config est repoussee a chaque
        connexion et pas seulement sur changement : une carte qui a redemarre seule est
        revenue a ses valeurs d'amorcage compilees.
        """
        if geom:
            # Unites du contrat MAVLink : MILLIMETRES BRUTS (pas le x10 du protocole
            # Yahboom). APB = DEMI-voie, le firmware la redouble
            # (kinematics.setWheelsYDistance(2 * apb_mm / 1000), firmware.cpp:597-618).
            circ_mm = math.pi * self.wheel_diameter * 1000.0
            apb_mm = 0.5 * self.wheel_sep * 1000.0
            ok, err = self.link.setWheelGeom(self.cpr, circ_mm, apb_mm, save=False)
            if not ok:
                self.get_logger().error(
                    f"ecriture geometrie refusee : {err or 'port indisponible'}")
            self.link.setCarType(int(round(self.car_type)), save=False)
            # La couche hote porte SA PROPRE copie du cpr (utilisee par _tick pour les rpm
            # et l'odom) : sans cette ligne, l'hote resterait sur l'ancienne echelle alors
            # que la carte, elle, aurait change -- desaccord silencieux entre les deux bouts.
            self.link.cpr = self.cpr
        if pid:
            kp, ki, kd = self.pid
            # motor_id=0 : les 4 moteurs. Sur cette carte ils PARTAGENT de toute facon un
            # seul triplet (repliement d'index idx % 3), contrairement au STM32.
            self.link.setMotorPid(kp, ki, kd, save=False, motor_id=0)
        if log_level:
            ok, err = self.link.setBoardLogLevel(self.board_log_level)
            if not ok:
                self.get_logger().warn(f"seuil de journal non ecrit : {err}")
        self._readback_board_config()

    def _readback_board_config(self):
        """Relit geometrie et PID DANS la carte : seule preuve que l'ecriture a porte.

        BLOQUANT (une requete puis l'attente du rapport, cf. RobotComSerial._requestReport)
        -> appele uniquement a l'init et sur changement de parametre, jamais dans la boucle
        de publication.
        """
        t = self.readback_timeout
        geom = self.link.getWheelGeom(timeout=t)
        pid = self.link.getPid(1, timeout=t)
        if geom is None or pid is None:
            self.board_cfg = None
            self.get_logger().warn(
                "relecture de la configuration carte SANS REPONSE : la valeur reellement "
                "en vigueur dans la SRAM est inconnue (board_config_ok=false).")
            return
        # Cles telles que produites par le codec (lisibles, destinees aussi au MCP).
        self.board_cfg = {
            "cpr": float(geom.get("cpr (tics/tour)") or 0.0),
            "circ_mm": float(geom.get("circ (mm)") or 0.0),
            "apb_mm": float(geom.get("APB (mm)") or 0.0),
            "kp": float(pid.get("kp") or 0.0),
            "ki": float(pid.get("ki") or 0.0),
            "kd": float(pid.get("kd") or 0.0),
        }
        self.get_logger().info(
            "carte relue : cpr={cpr:.0f} circ={circ_mm:.1f}mm APB={apb_mm:.1f}mm "
            "pid=({kp:.3f}, {ki:.3f}, {kd:.3f})".format(**self.board_cfg))

    def _drain_board_logs(self):
        """Republie le journal de la carte dans /rosout via le logger du noeud.

        La carte filtre deja A LA SOURCE selon LOG_LEVEL : ce qui arrive ici a donc passe
        son seuil, on ne refiltre pas. drainLogs() VIDE la file -> un seul consommateur.
        """
        for _t, sev, text in self.link.drainLogs():
            line = f"[carte] {text}"
            if sev <= 3:                       # EMERGENCY / ALERT / CRITICAL / ERROR
                self.get_logger().error(line)
            elif sev == 4:                     # WARNING
                self.get_logger().warn(line)
            elif sev <= 6:                     # NOTICE / INFO
                self.get_logger().info(line)
            else:                              # DEBUG
                self.get_logger().debug(line)

    # --- reconfiguration a chaud -------------------------------------------
    # Parametres diffuses vers la carte, groupes par trame a emettre.
    _GEOM_PARAMS = ("counts_per_rev", "wheel_diameter_m", "wheel_separation_m", "car_type")
    _PID_PARAMS = ("pid_kp", "pid_ki", "pid_kd")
    # Grandeurs qui n'ont aucun sens nulle ou negative (validation commune).
    _POSITIVE_PARAMS = _GEOM_PARAMS + _PID_PARAMS + (
        "motor_max_rpm", "motor_operating_voltage", "motor_power_max_voltage")

    def _on_set_parameters(self, params):
        """Valide PUIS applique un `ros2 param set` (donc aussi un appel MCP).

        En Humble ce callback est appele AVANT que rclpy ne stocke la valeur, et les
        variantes `add_pre/post_set_parameters_callback` n'existent qu'a partir d'Iron :
        l'effet de bord doit donc etre produit ICI meme, a partir de la liste recue -- et
        non de get_parameter(), qui renverrait encore l'ancienne valeur.

        Deux temps stricts : on valide TOUT, puis on applique. Un lot partiellement rejete
        laisserait la carte dans un etat que plus aucun parametre ROS ne decrit.
        """
        new = {}
        for prm in params:
            name, value = prm.name, prm.value
            if name in self._POSITIVE_PARAMS:
                if value is None or float(value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{name} doit etre strictement positif (recu {value}).")
            if name == "car_type" and not 1 <= int(round(float(value))) <= 6:
                return SetParametersResult(
                    successful=False,
                    reason="car_type hors enumere 1..6 (4 = FOURWHEEL pour ce robot).")
            if name == "board_log_level" and not 0 <= int(value) <= 7:
                return SetParametersResult(
                    successful=False, reason="board_log_level hors MAV_SEVERITY 0..7.")
            if name in ("encoder_left_index", "encoder_right_index") and not 0 <= int(value) <= 3:
                return SetParametersResult(
                    successful=False, reason=f"{name} hors plage 0..3 (voies M1..M4).")
            if name == "cmd_vel_timeout_s" and float(value) <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason="cmd_vel_timeout_s doit etre > 0 : un watchdog nul ne freine jamais.")
            if name == "board_readback_timeout_s" and float(value) <= 0.0:
                return SetParametersResult(
                    successful=False, reason="board_readback_timeout_s doit etre > 0.")
            new[name] = value

        # --- application : plus aucune validation ne peut echouer a partir d'ici ---
        if "counts_per_rev" in new:
            self.cpr = float(new["counts_per_rev"])
        if "wheel_diameter_m" in new:
            self.wheel_diameter = float(new["wheel_diameter_m"])
        if "wheel_separation_m" in new:
            self.wheel_sep = float(new["wheel_separation_m"])
        if "car_type" in new:
            self.car_type = float(new["car_type"])
        if any(k in new for k in self._PID_PARAMS):
            kp, ki, kd = self.pid
            self.pid = (float(new.get("pid_kp", kp)), float(new.get("pid_ki", ki)),
                        float(new.get("pid_kd", kd)))
        if "motor_max_rpm" in new:
            self.motor_max_rpm = float(new["motor_max_rpm"])
        if "motor_operating_voltage" in new:
            self.motor_voltage = float(new["motor_operating_voltage"])
        if "motor_power_max_voltage" in new:
            self.motor_max_voltage = float(new["motor_power_max_voltage"])
        if "encoder_left_index" in new:
            self.left_idx = int(new["encoder_left_index"])
        if "encoder_right_index" in new:
            self.right_idx = int(new["encoder_right_index"])
        if "invert_left" in new:
            self.inv_left = bool(new["invert_left"])
        if "invert_right" in new:
            self.inv_right = bool(new["invert_right"])
        if "cmd_vel_timeout_s" in new:
            self.cmd_vel_timeout = float(new["cmd_vel_timeout_s"])
        if "board_readback_timeout_s" in new:
            self.readback_timeout = float(new["board_readback_timeout_s"])
        if "board_log_level" in new:
            self.board_log_level = int(new["board_log_level"])
        if "publish_odom_tf" in new:
            self.publish_odom_tf = bool(new["publish_odom_tf"])
            # Un TransformBroadcaster ne se detruit pas proprement -> on le cree a la
            # demande et on se contente ensuite de ne plus l'utiliser (cf. _tick).
            if self.publish_odom_tf and self.tf_broadcaster is None:
                self.tf_broadcaster = TransformBroadcaster(self)

        touch_geom = any(k in new for k in self._GEOM_PARAMS)
        touch_pid = any(k in new for k in self._PID_PARAMS)
        touch_log = "board_log_level" in new
        if touch_geom or touch_pid or touch_log:
            self._push_board_config(geom=touch_geom, pid=touch_pid, log_level=touch_log)

        # L'armement EN DERNIER : si une autre valeur du meme lot avait ete refusee, on
        # n'aurait pas arme un robot sur une configuration a moitie appliquee.
        if "enable_cmd_vel" in new:
            self._apply_actuation(bool(new["enable_cmd_vel"]))

        return SetParametersResult(successful=True)

    def _apply_actuation(self, enable):
        """Cree ou detruit la souscription /cmd_vel. Idempotent.

        La souscription n'existait auparavant que si le parametre etait vrai AU DEMARRAGE :
        l'actuation n'etait donc pas armable a chaud, et l'orchestrateur MCP ne sachant pas
        passer d'argument de launch, elle n'etait pas armable depuis MCP du tout.
        """
        if enable and self.sub_cmd is None:
            self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
            self.get_logger().warn(
                "enable_cmd_vel=TRUE : ACTUATION ARMEE -> roues surelevees et geometrie "
                "calibree requises (lot 4).")
        elif not enable and self.sub_cmd is not None:
            self.destroy_subscription(self.sub_cmd)
            self.sub_cmd = None
            # Freiner AVANT de rendre la main : desarmer en laissant une derniere consigne
            # en vigueur sur la carte laisserait le robot rouler, le flux /cmd_vel ayant
            # justement disparu avec la souscription.
            self._brake()
            self.get_logger().info("enable_cmd_vel=false : actuation coupee, arret envoye.")
        elif not enable:
            self.get_logger().info(
                "enable_cmd_vel=false : LECTURE SEULE, aucune commande moteur emise.")
        self.enable_cmd_vel = bool(enable)
        self._last_cmd_t = None

    def _brake(self):
        """Arret REEL de la carte.

        `link.stop()` ne convient PAS ici : il emet BAMBOO_MOTOR_PWM, dont le `case` est
        VIDE dans ConnectorMavlink.cpp -- l'appel part, la carte l'ignore, rien ne freine.
        Le seul frein effectif sur cette carte est une consigne de vitesse nulle, repetee
        contre la perte d'une trame sur un UART partage avec la telemetrie.
        """
        for _ in range(3):
            self.link.sendCmdVel(0.0, 0.0)

    # --- actuation (L6 uniquement) -----------------------------------------
    def _on_cmd_vel(self, msg):
        if not self.enable_cmd_vel:
            return
        self.link.sendCmdVel(msg.linear.x, msg.angular.z)
        self._last_cmd_t = self.get_clock().now()

    # --- boucle de publication ---------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        snap = self.link.snapshot()
        sp = self.link.encSpeed()
        cpr = self.link.cpr or self.cpr

        # watchdog cmd_vel : coupe si le flux de consignes se tait (actuation seulement).
        # On freine UNE fois puis on oublie l'horodatage : sans ce reset, un robot a l'arret
        # reemettait une consigne nulle 30 fois par seconde pour rien, sur un UART partage
        # avec la telemetrie. Le prochain /cmd_vel rearme le watchdog.
        if self.enable_cmd_vel and self._last_cmd_t is not None:
            if (now - self._last_cmd_t).nanoseconds * 1e-9 > self.cmd_vel_timeout:
                self._brake()
                self._last_cmd_t = None
                self.get_logger().warn("watchdog cmd_vel : flux de consignes perdu -> arret.")

        # rpm par moteur (tics/s / cpr * 60), cf. BoardNode.
        if sp is not None:
            rpm_msg = WheelRpm()
            rpm_msg.header.stamp = stamp
            rpm_msg.rpm = [s / cpr * 60.0 for s in sp]
            self.pub_rpm.publish(rpm_msg)

        # odom integree depuis les 2 voies encodeur.
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if sp is not None and dt > 0.0:
            dist_per_tic = math.pi * self.wheel_diameter / cpr
            v_l = sp[self.left_idx] * dist_per_tic * (-1.0 if self.inv_left else 1.0)
            v_r = sp[self.right_idx] * dist_per_tic * (-1.0 if self.inv_right else 1.0)
            v = 0.5 * (v_l + v_r)
            w = (v_r - v_l) / self.wheel_sep if self.wheel_sep > 1e-6 else 0.0
            self.th += w * dt
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt
            qx, qy, qz, qw = _quat_from_euler(0.0, 0.0, self.th)

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w
            # Covariances diagonales indicatives (odom roues seule : theta peu fiable).
            odom.pose.covariance[0] = odom.pose.covariance[7] = 0.001
            odom.pose.covariance[35] = 0.01
            odom.twist.covariance[0] = 0.001
            odom.twist.covariance[35] = 0.01
            self.pub_odom.publish(odom)

            # Les deux conditions : le broadcaster survit a un passage a false du parametre
            # (il ne se detruit pas proprement), c'est donc le drapeau qui fait foi.
            if self.publish_odom_tf and self.tf_broadcaster is not None:
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = self.odom_frame
                tf.child_frame_id = self.base_frame
                tf.transform.translation.x = self.x
                tf.transform.translation.y = self.y
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(tf)

        # imu/data : roll/pitch/yaw sont en DEGRES dans le snapshot -> conversion en rad.
        if snap.get("yaw") is not None:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.imu_frame
            r = math.radians(snap.get("roll") or 0.0)
            p = math.radians(snap.get("pitch") or 0.0)
            yv = math.radians(snap.get("yaw") or 0.0)
            qx, qy, qz, qw = _quat_from_euler(r, p, yv)
            imu.orientation.x, imu.orientation.y = qx, qy
            imu.orientation.z, imu.orientation.w = qz, qw
            # La carte ne remonte pas gyro/accel via snapshot -> covariances a -1
            # (convention ROS : donnee absente, l'EKF ignore ces champs).
            imu.angular_velocity_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0
            self.pub_imu.publish(imu)

        # batterie.
        if snap.get("battery") is not None:
            b = BatteryState()
            b.header.stamp = stamp
            b.voltage = float(snap["battery"])
            b.present = True
            self.pub_batt.publish(b)

        # magneto (optionnelle) : uT -> Tesla.
        mag = snap.get("mag")
        if mag is not None and len(mag) >= 3:
            m = MagneticField()
            m.header.stamp = stamp
            m.header.frame_id = self.imu_frame
            m.magnetic_field.x = float(mag[0]) * 1e-6
            m.magnetic_field.y = float(mag[1]) * 1e-6
            m.magnetic_field.z = float(mag[2]) * 1e-6
            self.pub_mag.publish(m)

        # etat brut agrege (diagnostic, observable directement via le MCP).
        st = Esp32Status()
        st.header.stamp = stamp
        st.battery_voltage = float(snap.get("battery") or 0.0)
        st.roll = float(snap.get("roll") or 0.0)
        st.pitch = float(snap.get("pitch") or 0.0)
        st.yaw = float(snap.get("yaw") or 0.0)
        st.vx = float(snap.get("vx") or 0.0)
        st.vy = float(snap.get("vy") or 0.0)
        st.vz = float(snap.get("vz") or 0.0)
        enc = snap.get("encoders")
        st.encoders = [int(e) for e in enc] if enc else []
        # Relecture de la SRAM de la carte : publiee a chaque cycle mais RAFRAICHIE
        # uniquement lors d'une reconfiguration (la relecture est bloquante). C'est donc un
        # etat, pas une mesure -- et board_config_ok=false dit que la carte n'a pas repondu.
        cfg = self.board_cfg
        st.board_config_ok = cfg is not None
        if cfg is not None:
            st.board_cpr = cfg["cpr"]
            st.board_circ_mm = cfg["circ_mm"]
            st.board_apb_mm = cfg["apb_mm"]
            st.board_pid_kp = cfg["kp"]
            st.board_pid_ki = cfg["ki"]
            st.board_pid_kd = cfg["kd"]
        self.pub_status.publish(st)

    def destroy_node(self):
        try:
            # _brake() et NON link.stop() : ce dernier est un no-op sur cette carte, donc
            # l'arret du noeud laissait le robot rouler sur sa derniere consigne.
            if self.enable_cmd_vel:
                self._brake()
            self.link.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32MavlinkDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
