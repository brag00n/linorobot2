#!/usr/bin/env python3
r"""stm32_mavlink_driver - pont MAVLink Yahboom STM32 <-> ROS2 (proprietaire UNIQUE du serie).

Module de controleur `bamboo_controler_YBStm32v3` : le driver COMPLET de la Yahboom ROS Robot
Control Board V3.0 (STM32F103RCT6, MAVLink v2 dialecte bamboo, sysid 1, CH340 1A86:7523 a
115200 bauds). Il implemente TOUT ce que cette carte sait mapper vers ROS -- motorisation,
IMU, magnetometre, batterie, encodeurs, servos, journal, parametres -- et pas seulement ce
dont le tracking camera a besoin. Un driver partiel obligerait a le rouvrir a chaque nouvel
usage, et surtout laisserait croire que la carte ne sait pas faire le reste.

Reutilise TEL QUEL la couche hote `robot_control` : RobotComSerial ouvre le port dans un
thread lecteur, decode le protocole et expose snapshot() / encSpeed() / setWheelGeom() /
setMotorPid() / sendServo() / drainLogs(). AUCUN protocole n'est reecrit ici.

MEME PHILOSOPHIE DE MAPPAGE que esp32_mavlink_driver (patron explicite) : parametres physiques
declares SANS DEFAUT et pris dans la source unique config/robots/<robot>.yaml, reconfiguration
A CHAUD par `ros2 param set` avec effet de bord DANS le callback (contrainte Humble), diffusion
vers la SRAM de la carte puis RELECTURE de verification publiee dans Stm32Status, verrou
d'actuation commutable a chaud.

QUATRE ECARTS DE FOND avec le patron ESP32, tous dictes par la carte :

  1. GAINS PID PAR MOTEUR. Le firmware adresse chaque moteur (repliement idx / 3,
     mav_protocol.c:143-153) la ou l'ESP32 replie les 4 moteurs sur un triplet unique
     (idx % 3). Les parametres pid_kp/ki/kd sont donc des TABLEAUX de 4, pousses par
     setMotorPid(..., motor_id=1..4) et relus par getPid(1..4). getPid(5) serait le PID de
     CAP (yaw), un asservissement distinct -- ne pas l'inclure dans la boucle des roues.

  2. PERSISTANCE FLASH REELLE. save=false partout par defaut : la persistance est un ACTE
     VOLONTAIRE, expose par le service `save_board_config`, jamais un effet de bord d'un
     `ros2 param set`. Sur l'ESP32 la question ne se posait pas (aucune persistance).

  3. `board_log_level` EST UN FILTRE D'HOTE, PAS UN REGLAGE DE CARTE. La table PARAM du STM32
     s'arrete a l'index 18 (P_CAR_TYPE, PARAM_COUNT 19) : elle n'a pas le LOG_LEVEL idx 19 de
     l'ESP32. Ecrire ce seuil reviendrait a pousser un index que la carte ignore, avec un echo
     satisfait -- exactement le genre de reglage qui semble pris et ne fait rien. Le filtrage
     se fait donc DANS _drain_board_logs.
     CAVEAT HONNETE : filtrer cote hote menage le journal ROS mais PAS l'UART -- la carte emet
     tout, le trafic reste entier. Pour menager le lien il faudrait ajouter l'index 19 au
     firmware (reflash par IAP, possible sans BOOT0/RESET).

  4. LA SENTINELLE D'ESCLAVAGE N'EST PAS ECRIVABLE D'ICI, et c'est une LIMITE DE LA VOIE
     MAVLINK, pas un etat de ce robot. Le firmware sait desactiver le PID d'un moteur et le
     caler en recopie sur son voisin de meme cote (F_PID_SLAVED_MARK, app_flash.h:57) -- c'est
     la reponse a une voie encodeur morte. Mais setMotorPid(disable=True) n'existe QUE dans le
     protocole Yahboom : le codec MAVLink l'ignore. Ce driver ne peut donc ni poser ni retirer
     la sentinelle ; il faudrait passer par l'outillage MCP `robot-action`.
     ETAT ACTUEL : elle n'est PAS posee. La carte a ete remplacee (2026-09-25), les quatre
     voies encodeur sont saines et les quatre moteurs aussi -- les quatre triplets de gains
     pousses d'ici sont donc tous ACTIFS. Aucun contournement de panne ne subsiste ici, et il
     ne doit pas en etre reintroduit : si une voie tombe un jour, c'est un acte explicite et
     journalise, pas un index traite a part en silence.

VERROU D'ACTUATION : `enable_cmd_vel` est FALSE par defaut, et ce n'est pas de la prudence de
principe. Les constantes PWM du firmware ne sont pas mises a l'echelle -- le pack 12,6 V
arrive BRUT sur des moteurs 7,4 V nominaux -- et les essais T2/T3/T5 de bambooSTM32YB ne sont
pas passes. La CAPACITE moteur est entierement implementee ; son ARMEMENT attend la
calibration. LES SERVOS OUI, LES MOTEURS NON.

SERVOS : ce noeud est UN EXECUTEUR du contrat ROS /servo/cmd, pas le proprietaire de la notion.
Il souscrit parce qu'il DECLARE la capacite servo, resout `axis` en voie materielle par la
table `servo_axes` du fichier canonique, borne, et journalise tout ecretage. Rien du groupe
tracking ne sait que c'est lui. Deux asymetries de CETTE carte sont absorbees ICI et surtout
pas chez le producteur : l'hote BORNE (RobotComSerial.py:449) alors que le firmware JETTE EN
SILENCE au-dela de 180 deg (bsp_pwmServo.c:153), et un COMMAND_ACK renvoie MAV_RESULT_ACCEPTED
MEME quand la valeur a ete jetee -> un ACK ne prouve PAS l'application. L'angle est en ECRITURE
SEULE (PwmServo_Get_Angle sans appelant, pas de SERVO_OUTPUT_RAW, pas d'index PARAM servo) ->
/servo/state et Stm32Status.servo_angle_deg sont des OMBRES D'HOTE, nommees comme telles.

PROPRIETAIRE UNIQUE DU SERIE : ce noeud, l'application robot_controlv3 et le serveur MCP
`robot-action` sont MUTUELLEMENT EXCLUSIFS sur /dev/stm32. Arreter l'un avant de lancer l'autre.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState, MagneticField, JointState
from geometry_msgs.msg import Twist, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from bamboo_interfaces.msg import WheelRpm, Stm32Status, ServoCmd
from bamboo_interfaces.srv import Nudge

from robot_control.communication.RobotComSerial import RobotComSerial


# Ordre des articulations = ordre des moteurs M1..M4 de la carte. Noms de l'URDF 4wd.
# ATTENTION au cablage de CE robot : la marche avant demande M1+ M2- M3- M4+, donc les cotes
# sont {M1, M3} et {M2, M4} avec un moteur inverse dans chaque paire. Les noms ci-dessous
# suivent l'URDF amont, pas le signe du cablage -- qu'on ne "corrige" PAS cote ROS (une
# inversion d'hote desynchroniserait le PID embarque et l'odometrie).
JOINT_NAMES = ("front_left_wheel_joint", "front_right_wheel_joint",
               "rear_left_wheel_joint", "rear_right_wheel_joint")

# Identifiant de ce module de controleur, tel qu'ecrit dans `servo_axes[*].controller` du
# fichier canonique. C'est la seule chaine qui lie ce fichier-ci a la table d'axes.
CONTROLLER_ID = "YBStm32v3"

# Voies servo PWM de la carte (headers S1..S4, id MAVLink 1..4). Les servos de BUS serie
# (app_uart_servo.c, FUNC 0x20-0x24) sont une autre chose : c'est la chaine du bras robotise
# Yahboom, sans objet pour des SG90 sur les headers PWM.
PWM_SERVO_COUNT = 4


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


class Stm32MavlinkDriver(Node):
    # Parametres physiques attendus de la source unique config/robots/<robot>.yaml.
    # Aucun n'a de defaut : voir _req().
    _PHYS_DOUBLES = ("car_type", "counts_per_rev", "wheel_diameter_m", "wheel_separation_m",
                     "motor_max_rpm", "motor_operating_voltage", "motor_power_max_voltage",
                     "yaw_pid_kp", "yaw_pid_ki", "yaw_pid_kd")
    # ECART AVEC L'ESP32 : QUATRE triplets, un par moteur (cf. en-tete, ecart 1).
    _PHYS_DOUBLE_ARRAYS = ("pid_kp", "pid_ki", "pid_kd")
    _PHYS_INTS = ("encoder_left_index", "encoder_right_index")
    _PHYS_BOOLS = ("invert_left", "invert_right")

    def _req(self, name):
        """Lit un parametre physique OBLIGATOIRE ; leve si la source unique ne l'a pas fourni.

        Un placeholder silencieux est exactement ce qui a produit quatre diametres de roue
        contradictoires dans ce depot : l'absence doit etre une erreur FATALE et NOMMEE.
        """
        try:
            value = self.get_parameter(name).value
        except Exception:
            value = None
        if value is None:
            raise RuntimeError(
                f"parametre physique obligatoire absent : '{name}'. Il doit venir du fichier "
                "canonique bamboo_base/config/robots/<robot>.yaml (charge par "
                "driver.launch.py) ; ce noeud ne porte volontairement aucun defaut physique.")
        return value

    def _declare_phys(self, name, ptype):
        """Declare un parametre physique SANS valeur, sauf s'il est deja arrive par override.

        Le noeud active la declaration automatique depuis les overrides (seul moyen public de
        lire une carte YAML IMBRIQUEE comme `servo_axes`, cf. _load_servo_axes) : les
        parametres presents dans le fichier canonique sont donc DEJA declares ici, et les
        re-declarer leverait. Ce qui reste utile, c'est de declarer le TYPE des ABSENTS, pour
        que _req() echoue en les NOMMANT plutot que sur une exception opaque.
        """
        if not self.has_parameter(name):
            self.declare_parameter(name, ptype)

    def _declare_host(self, name, default, descriptor=None):
        """Declare un parametre d'hote, en preservant la valeur venue d'un override.

        La declaration automatique ne pose AUCUN descripteur : un `port` venu du YAML serait
        donc modifiable a chaud alors qu'il est structurellement fige. On le re-declare avec
        son descripteur en reprenant sa valeur -- sinon `read_only` serait perdu pour tout
        parametre effectivement configure, c'est-a-dire precisement ceux qui comptent.
        """
        if self.has_parameter(name):
            value = self.get_parameter(name).value
            self.undeclare_parameter(name)
        else:
            value = default
        self.declare_parameter(name, value, descriptor or ParameterDescriptor())

    def __init__(self):
        # automatically_declare_parameters_from_overrides : indispensable pour `servo_axes`,
        # qui arrive en parametres POINTES (servo_axes.pan.channel, ...) dont on ne connait pas
        # les noms d'avance -- aucun get_parameter unique ne lit une carte imbriquee. La
        # contrepartie est geree par _declare_phys / _declare_host.
        super().__init__("stm32_mavlink_driver",
                        automatically_declare_parameters_from_overrides=True)

        # --- transport et frames (faits d'HOTE : un defaut y est legitime) ---
        # read_only : figes STRUCTURELLEMENT des que le noeud tourne (le port est ouvert par un
        # thread lecteur dans le constructeur, la periode du timer est capturee a sa creation).
        # Un `ros2 param set` est alors REJETE avec un motif, au lieu d'etre accepte sans effet
        # -- un reglage qui semble pris mais ne fait rien coute plus cher a diagnostiquer.
        _fixed = ParameterDescriptor(
            read_only=True,
            description="fige au demarrage : relancer le noeud pour en changer")
        self._declare_host("port", "/dev/stm32", _fixed)
        self._declare_host("baud", 115200, _fixed)
        self._declare_host("publish_rate_hz", 30.0, _fixed)

        # --- parametres PHYSIQUES : declares SANS DEFAUT ---
        for _name in self._PHYS_DOUBLES:
            self._declare_phys(_name, Parameter.Type.DOUBLE)
        for _name in self._PHYS_DOUBLE_ARRAYS:
            self._declare_phys(_name, Parameter.Type.DOUBLE_ARRAY)
        for _name in self._PHYS_INTS:
            self._declare_phys(_name, Parameter.Type.INTEGER)
        for _name in self._PHYS_BOOLS:
            self._declare_phys(_name, Parameter.Type.BOOL)
        self._declare_phys("robot_base", Parameter.Type.STRING)

        self._declare_host("odom_frame", "odom")
        self._declare_host("base_frame", "base_footprint")
        self._declare_host("imu_frame", "imu_link")
        # L'EKF possede le TF odom->base_footprint -> le driver ne le publie PAS par defaut.
        self._declare_host("publish_odom_tf", False)
        # Verrou d'actuation MOTEUR, commutable a chaud (cf. _apply_actuation) : c'est le canal
        # d'armement depuis MCP, l'orchestrateur ne sachant pas passer d'argument de launch.
        # Defaut false, non negociable sur ce robot (cf. en-tete).
        self._declare_host("enable_cmd_vel", False)
        self._declare_host("cmd_vel_timeout_s", 0.25)
        # Recentrage des servos au demarrage. Defaut FALSE : un noeud qui bouge la camera des
        # qu'il demarre produit un mouvement que personne n'a demande. A passer true une fois
        # les butees validees (T6 de bambooSTM32YB).
        self._declare_host("servo_center_on_start", False)

        # --- fiabilite QoS des capteurs ---
        # `sensor_data` est BEST-EFFORT, donc INVISIBLE a travers rosbridge (qui souscrit en
        # RELIABLE) : l'IMU ne serait pas diagnosticable depuis le MCP. `reliable` la rend
        # lisible au prix d'un peu de retransmission -- acceptable a 30 Hz en local.
        self._declare_host("imu_qos", "reliable")

        # --- journal de la carte ---
        # FILTRE D'HOTE, pas un reglage de carte : cette carte n'a pas le PARAM LOG_LEVEL
        # idx 19 de l'ESP32 (table arretee a 18). Cf. en-tete, ecart 3, et son caveat UART.
        self._declare_host("board_log_level", 6)
        # Delai d'attente d'une RELECTURE. La relecture est bloquante (requete puis attente du
        # rapport) : elle suspend brievement la publication. Assume -- une reconfiguration est
        # un acte rare et delibere, et la preuve que la carte a pris la valeur vaut cette
        # pause. La carte repond d'ordinaire en ~50 ms.
        self._declare_host("board_readback_timeout_s", 1.0)

        g = lambda n: self.get_parameter(n).value
        self.port = g("port")
        self.baud = int(g("baud"))
        rate = float(g("publish_rate_hz"))
        self.robot_base = str(self._req("robot_base"))
        self.car_type = float(self._req("car_type"))
        self.cpr = float(self._req("counts_per_rev"))
        self.wheel_diameter = float(self._req("wheel_diameter_m"))
        self.wheel_sep = float(self._req("wheel_separation_m"))
        # Motorisation : lue ici pour echouer tot si la source est incomplete. ATTENTION, ces
        # deux tensions ne sont CONSOMMEES PAR RIEN sur cette carte -- la cinematique et le PID
        # sont embarques et travaillent en mm/s et tics/s, il n'y a aucun equivalent du
        # `supply = constrain(...)` de l'ESP32. Elles sont lues parce qu'un fait physique
        # manquant doit se voir, pas parce qu'un calcul les utilise.
        self.motor_max_rpm = float(self._req("motor_max_rpm"))
        self.motor_voltage = float(self._req("motor_operating_voltage"))
        self.motor_max_voltage = float(self._req("motor_power_max_voltage"))
        # QUATRE triplets, un par moteur (cf. en-tete, ecart 1).
        self.pid_kp = [float(v) for v in self._req("pid_kp")]
        self.pid_ki = [float(v) for v in self._req("pid_ki")]
        self.pid_kd = [float(v) for v in self._req("pid_kd")]
        self._check_pid_shape()
        # PID de CAP : asservissement distinct du PID roue (index 5 cote getPid). Lu pour la
        # meme raison que les tensions ; son ECRITURE n'a pas d'equivalent dans la couche hote
        # en MAVLink (setYawPid est une voie Yahboom) -> declare, NON pousse. Dit ici plutot
        # que decouvert a l'execution.
        self.yaw_pid = (float(self._req("yaw_pid_kp")), float(self._req("yaw_pid_ki")),
                        float(self._req("yaw_pid_kd")))
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
        # Derniere relecture reussie de la SRAM de la carte, publiee dans Stm32Status.
        self.board_cfg = None

        # --- table d'axes servo (le seul lien axe -> voie materielle) ---
        self.axes = self._load_servo_axes()

        # --- lien serie (proprietaire unique) ---
        # vid_pid CH340 : cette carte est seule de son espece sur ce banc (contrairement au
        # double CP2102 lidar/ESP32), donc le VID:PID la designe sans ambiguite.
        self.link = RobotComSerial(self.port, self.baud, protocol="mavlink",
                                   cpr=self.cpr, vid_pid=["1A86:7523"], telemetry=None)

        # --- publishers ---
        self.pub_odom = self.create_publisher(Odometry, "odom/unfiltered", 20)
        # QoS resolue une fois : un publisher ne change pas de fiabilite en marche (il faudrait
        # le detruire et le recreer, ce qui coupe les souscripteurs en place).
        _imu_qos = 10 if self.imu_qos_name == "reliable" else qos_profile_sensor_data
        self.pub_imu = self.create_publisher(Imu, "imu/data", _imu_qos)
        self.pub_batt = self.create_publisher(BatteryState, "battery", 10)
        self.pub_rpm = self.create_publisher(WheelRpm, "wheel_rpm", 10)
        # Le MPU9250 a un MAGNETOMETRE -> un cap ABSOLU est possible sur cette carte, ce qui
        # est impossible sur le MPU6050 du Teensy et de la GrovePi+.
        self.pub_mag = self.create_publisher(MagneticField, "mag", _imu_qos)
        self.pub_status = self.create_publisher(Stm32Status, "stm32_status", 10)
        self.pub_joint = self.create_publisher(JointState, "joint_states", 10)
        self.pub_joint_req = self.create_publisher(JointState, "req_states", 10)
        # OMBRE D'HOTE : ce qu'on a EMIS, jamais ce que le servo a ATTEINT. Profondeur 1 pour
        # qu'un consommateur tardif lise la consigne courante sans attendre le prochain
        # mouvement.
        self.pub_servo = self.create_publisher(ServoCmd, "servo/state", 1)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None

        # --- souscription cmd_vel (GATEE par enable_cmd_vel, commutable a chaud) ---
        self._last_cmd_t = None
        self.sub_cmd = None
        self._apply_actuation(self.enable_cmd_vel)

        # --- servos : souscription et services crees PARCE QUE cette carte declare la
        # capacite servo. Un module de controleur qui ne l'a pas (WSEsp32 : DO_SET_SERVO
        # repond UNSUPPORTED) ne doit rien creer ici -- une capacite absente doit etre ABSENTE
        # de l'interface, pas decouverte a l'execution.
        self.sub_servo = self.create_subscription(ServoCmd, "servo/cmd", self._on_servo_cmd, 10)
        self.srv_nudge = self.create_service(Nudge, "servo/nudge", self._on_nudge)
        # Persistance flash : ACTE VOLONTAIRE, jamais un effet de bord d'un `ros2 param set`.
        self.srv_save = self.create_service(Trigger, "save_board_config", self._on_save)

        # --- etat de l'odometrie integree ---
        self.x = self.y = self.th = 0.0
        self._last_t = self.get_clock().now()

        # --- diffusion de la configuration vers la SRAM de la carte ---
        # A L'INIT et pas seulement sur changement : cette carte PERSISTE en flash, donc elle
        # a pu redemarrer sur une configuration ecrite lors d'une session precedente qui ne
        # decrit plus ce que le fichier canonique dit aujourd'hui. Le fichier canonique fait
        # foi -> on le repousse a chaque connexion, puis on relit pour le prouver.
        # save=False : ce push ne touche PAS la flash.
        self._push_board_config(geom=True, pid=True)

        # Journal draine a 2 Hz : ces lignes sont rares et la file hote est bornee (200),
        # inutile de payer ce parcours 30 fois par seconde.
        self.log_timer = self.create_timer(0.5, self._drain_board_logs)

        if bool(g("servo_center_on_start")):
            self._center_servos()

        # Enregistre EN DERNIER : le callback touche a des membres et au lien serie, il ne doit
        # pas pouvoir s'executer sur un objet a demi construit.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"driver STM32 MAVLink demarre (port={self.port}, baud={self.baud}, "
            f"cpr={self.cpr}, axes servo={sorted(self.axes)}).")

    # --- coherence de la configuration ------------------------------------
    def _check_pid_shape(self):
        """Les trois tableaux de gains doivent decrire les 4 moteurs, ni plus ni moins.

        Un tableau a 3 elements laisserait croire a une carte a trois moteurs et decalerait
        silencieusement les gains ; un tableau a 1 element ferait passer cette carte pour
        l'ESP32 (triplet commun aux 4), ce qu'elle n'est pas.
        """
        for name, arr in (("pid_kp", self.pid_kp), ("pid_ki", self.pid_ki),
                          ("pid_kd", self.pid_kd)):
            if len(arr) != 4:
                raise RuntimeError(
                    f"'{name}' doit porter 4 valeurs (un gain par moteur M1..M4 : cette carte "
                    f"adresse chaque moteur, repliement idx / 3), or il en porte {len(arr)}.")

    def _load_servo_axes(self):
        """Resout `servo_axes` (carte YAML imbriquee) en table axe -> voie materielle.

        Les cartes imbriquees arrivent en parametres POINTES : servo_axes.pan.channel, etc.
        get_parameters_by_prefix les regroupe -- c'est le seul moyen public de les lire, et la
        raison pour laquelle ce noeud active la declaration automatique depuis les overrides.

        N'est retenu que ce qui porte `controller: YBStm32v3`. Un axe confie a une AUTRE carte
        est legitime et n'est pas une erreur ici : il est simplement execute ailleurs. Ce que
        ce noeud refuse, c'est un axe QUI LUI EST CONFIE et dont la description est incomplete
        -- la, il echoue EN NOMMANT L'AXE, plutot que d'avaler ses consignes en silence.
        """
        raw = {}
        for suffix, param in self.get_parameters_by_prefix("servo_axes").items():
            axis, _, leaf = suffix.partition(".")
            if not leaf:
                continue
            raw.setdefault(axis, {})[leaf] = param.value
        axes, foreign = {}, []
        for axis, spec in sorted(raw.items()):
            if str(spec.get("controller", "")) != CONTROLLER_ID:
                foreign.append(f"{axis}->{spec.get('controller')}")
                continue
            missing = [k for k in ("channel", "min_deg", "max_deg", "rest_deg")
                       if spec.get(k) is None]
            if missing:
                raise RuntimeError(
                    f"axe servo '{axis}' confie a {CONTROLLER_ID} mais incomplet : champs "
                    f"manquants {missing} dans servo_axes du fichier canonique.")
            channel = int(spec["channel"])
            if not 1 <= channel <= PWM_SERVO_COUNT:
                raise RuntimeError(
                    f"axe servo '{axis}' : channel {channel} hors des voies PWM S1..S"
                    f"{PWM_SERVO_COUNT} de cette carte.")
            lo, hi = float(spec["min_deg"]), float(spec["max_deg"])
            if lo >= hi:
                raise RuntimeError(f"axe servo '{axis}' : min_deg ({lo}) >= max_deg ({hi}).")
            axes[axis] = {"channel": channel, "min": lo, "max": hi,
                          "rest": min(max(float(spec["rest_deg"]), lo), hi)}
        if foreign:
            self.get_logger().info(
                "axes servo confies a un autre controleur, non executes ici : "
                + ", ".join(foreign))
        # OMBRE D'HOTE par voie S1..S4. NaN = "jamais commandee" : un zero ferait croire a un
        # angle de 0 deg, qui est une position reelle et hors des bornes de ce robot.
        self._servo_shadow = [float("nan")] * PWM_SERVO_COUNT
        return axes

    # --- diffusion vers la carte -------------------------------------------
    def _push_board_config(self, geom=False, pid=False, save=False):
        """Ecrit la configuration dans la carte, puis la RELIT pour verification.

        `save=False` par defaut, et c'est le point de fond : contrairement a l'ESP32, cette
        carte SAIT persister en flash. Ecrire la flash a chaque `ros2 param set` userait
        inutilement le secteur et, pire, ferait redemarrer la carte sur un reglage d'essai.
        La persistance passe donc par le service `save_board_config`, explicitement.
        """
        if geom:
            # Unites du contrat MAVLink : MILLIMETRES BRUTS (pas le x10 du protocole Yahboom).
            # APB = DEMI-voie, le firmware la redouble
            # (kinematics.setWheelsYDistance(2 * apb_mm / 1000)).
            circ_mm = math.pi * self.wheel_diameter * 1000.0
            apb_mm = 0.5 * self.wheel_sep * 1000.0
            ok, err = self.link.setWheelGeom(self.cpr, circ_mm, apb_mm, save=save)
            if not ok:
                self.get_logger().error(
                    f"ecriture de la geometrie refusee : {err or 'port indisponible'}")
            self.link.setCarType(int(round(self.car_type)), save=save)
            # La couche hote porte SA PROPRE copie du cpr (elle s'en sert pour convertir les
            # tics) : sans cette ligne, hote et carte divergeraient silencieusement d'echelle.
            self.link.cpr = self.cpr
        if pid:
            # UN APPEL PAR MOTEUR (motor_id 1..4), pas un appel global : cette carte adresse
            # chaque moteur, contrairement a l'ESP32 qui replie les 4 sur un triplet unique.
            # Les quatre triplets portent reellement : aucune sentinelle d'esclavage n'est
            # posee sur ce robot, donc aucun index n'est a traiter a part.
            for i in range(4):
                if not self.link.setMotorPid(self.pid_kp[i], self.pid_ki[i], self.pid_kd[i],
                                             save=save, motor_id=i + 1):
                    self.get_logger().error(f"ecriture du PID moteur M{i + 1} refusee")
        self._readback_board_config()

    def _readback_board_config(self):
        """Relit geometrie et PID DANS la carte : seule preuve que l'ecriture a porte.

        BLOQUANT (une requete puis l'attente du rapport) -> appele uniquement a l'init et sur
        changement de parametre, jamais dans _tick.

        getPid(1..4) = les quatre moteurs. getPid(5) serait le PID de CAP, un asservissement
        distinct : l'inclure ici melangerait deux boucles differentes dans le meme champ.
        """
        t = self.readback_timeout
        geom = self.link.getWheelGeom(timeout=t)
        pids = [self.link.getPid(i, timeout=t) for i in range(1, 5)]
        if geom is None or any(p is None for p in pids):
            self.board_cfg = None
            self.get_logger().warn(
                "relecture de la configuration carte SANS REPONSE (ou partielle) : la valeur "
                "reellement en vigueur dans la SRAM est inconnue (board_config_ok=false).")
            return
        # Cles telles que produites par le codec (lisibles, destinees aussi au MCP).
        self.board_cfg = {
            "cpr": float(geom.get("cpr (tics/tour)") or 0.0),
            "circ_mm": float(geom.get("circ (mm)") or 0.0),
            "apb_mm": float(geom.get("APB (mm)") or 0.0),
            "kp": [float(p.get("kp") or 0.0) for p in pids],
            "ki": [float(p.get("ki") or 0.0) for p in pids],
            "kd": [float(p.get("kd") or 0.0) for p in pids],
        }

    def _drain_board_logs(self):
        """Republie le journal de la carte dans /rosout via le logger du noeud.

        FILTRAGE COTE HOTE, contrairement a l'ESP32 qui filtre A LA SOURCE : la table PARAM de
        cette carte s'arrete a l'index 18, elle n'a pas de LOG_LEVEL. Tout ce que la carte emet
        arrive donc ici, et c'est nous qui coupons au-dela de `board_log_level`.
        CAVEAT : ca menage le journal ROS, PAS l'UART -- le trafic serie reste entier.

        drainLogs() VIDE la file -> un seul consommateur, d'ou ce timer unique.
        """
        for _t, sev, text in self.link.drainLogs():
            if sev > self.board_log_level:
                continue
            line = f"[carte] {text}"
            if sev <= 3:                       # EMERGENCY / ALERT / CRITICAL / ERROR
                self.get_logger().error(line)
            elif sev == 4:                     # WARNING
                self.get_logger().warn(line)
            elif sev <= 6:                     # NOTICE / INFO
                self.get_logger().info(line)
            else:                              # DEBUG
                self.get_logger().debug(line)

    # --- servos -------------------------------------------------------------
    def _send_axis(self, axis, angle_deg, relative=False):
        """Applique une consigne a un axe NOMME. Renvoie (ok, angle_rendu, motif).

        C'est ici, dans l'executeur, que les asymetries de CETTE carte sont absorbees -- pas
        chez le producteur, qui ne doit rien savoir du materiel :
          - le bornage est la SEULE protection reelle : le firmware, lui, JETTE EN SILENCE
            au-dela de 180 deg (bsp_pwmServo.c:153) et l'ACK renvoie ACCEPTED quand meme ;
          - un ecretage est JOURNALISE, jamais avale : 200 deg devient 172 deg et se voit.
        La borne haute 172 n'est pas une marge de confort : la loi d'impulsion du firmware est
        (angle * 11 + 500) / 10 us, donc 180 deg -> 2480 us, soit 80 us AU-DELA de la butee du
        SG90 (bourdonnement, ~700 mA continus, pignons plastique).
        """
        spec = self.axes.get(axis)
        if spec is None:
            return (False, 0.0, f"axe inconnu de {CONTROLLER_ID} : '{axis}' "
                                f"(axes servis : {sorted(self.axes)})")
        cur = self._servo_shadow[spec["channel"] - 1]
        if relative:
            base = spec["rest"] if math.isnan(cur) else cur
            target = base + float(angle_deg)
        else:
            target = float(angle_deg)
        clamped = min(max(target, spec["min"]), spec["max"])
        if abs(clamped - target) > 1e-6:
            self.get_logger().warn(
                f"axe '{axis}' : consigne {target:.1f} deg ECRETEE a {clamped:.1f} deg "
                f"(bornes {spec['min']:.0f}-{spec['max']:.0f}).")
        if not self.link.sendServo(spec["channel"], clamped):
            return (False, clamped, f"envoi refuse sur la voie S{spec['channel']}")
        # Ombre d'hote mise a jour APRES un envoi accepte par la couche hote. Elle reste une
        # ombre : aucun retour de position n'existe sur cette carte.
        self._servo_shadow[spec["channel"] - 1] = clamped
        msg = ServoCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axis = axis
        msg.angle_deg = float(clamped)
        msg.relative = False
        msg.id = spec["channel"]
        self.pub_servo.publish(msg)
        return (True, clamped, "")

    def _on_servo_cmd(self, msg):
        """Consigne d'axe venue de servocam_node, de la manette ou du MCP.

        `msg.id` est deliberement IGNORE : c'est la table `servo_axes` qui resout le nom en
        voie, sinon un producteur qui remplirait la voie a la main recreerait le couplage carte
        que ce contrat existe pour retirer.
        """
        ok, _angle, reason = self._send_axis(msg.axis, msg.angle_deg, bool(msg.relative))
        if not ok:
            self.get_logger().warn(f"/servo/cmd rejete : {reason}")

    def _on_nudge(self, request, response):
        """Increments relatifs, ATOMIQUES : tout valide avant d'emettre quoi que ce soit.

        Le stick droit de la manette bouge pan ET tilt d'un coup ; appliquer le premier axe
        puis echouer sur le second laisserait la nacelle dans une pose que personne n'a
        demandee. On valide donc les deux listes d'abord.
        """
        if len(request.axis) != len(request.delta):
            response.success = False
            response.reason = (f"listes discordantes : {len(request.axis)} axes pour "
                               f"{len(request.delta)} increments")
            response.angle_deg = []
            return response
        unknown = [a for a in request.axis if a not in self.axes]
        if unknown:
            response.success = False
            response.reason = (f"axes inconnus de {CONTROLLER_ID} : {unknown} "
                               f"(axes servis : {sorted(self.axes)})")
            response.angle_deg = []
            return response
        angles, failed = [], ""
        for axis, delta in zip(request.axis, request.delta):
            ok, angle, reason = self._send_axis(axis, float(delta), relative=True)
            angles.append(float(angle))
            if not ok and not failed:
                failed = f"axe '{axis}' : {reason}"
        response.success = not failed
        response.reason = failed
        response.angle_deg = angles
        return response

    def _center_servos(self):
        """Ramene chaque axe servi a sa position de repos. Appele seulement sur demande.

        Sous `servo_center_on_start`, donc JAMAIS par defaut : un demarrage de conteneur ne
        doit pas produire de mouvement que personne n'a demande.
        """
        for axis, spec in sorted(self.axes.items()):
            ok, _angle, reason = self._send_axis(axis, spec["rest"])
            if not ok:
                self.get_logger().warn(f"recentrage de '{axis}' echoue : {reason}")

    def _on_save(self, request, response):
        """Persiste la configuration courante en FLASH (ce que l'ESP32 ne sait pas faire).

        Re-pousse geometrie, car type et PID avec `save=True` : la couche hote n'expose pas
        PREFLIGHT_STORAGE seul, la persistance est portee par le drapeau de chaque ecriture.
        Puis relit -- une sauvegarde non verifiee ne vaut pas mieux qu'une ecriture muette.
        """
        self._push_board_config(geom=True, pid=True, save=True)
        response.success = self.board_cfg is not None
        response.message = ("configuration ecrite en flash et relue" if response.success else
                            "ecriture tentee mais RELECTURE SANS REPONSE : la persistance "
                            "n'est pas prouvee")
        return response

    # --- parametres a chaud -------------------------------------------------
    _GEOM_PARAMS = ("counts_per_rev", "wheel_diameter_m", "wheel_separation_m", "car_type")
    _PID_PARAMS = ("pid_kp", "pid_ki", "pid_kd")
    _POSITIVE_PARAMS = _GEOM_PARAMS + (
        "motor_max_rpm", "motor_operating_voltage", "motor_power_max_voltage")

    def _on_set_parameters(self, params):
        """Point de jonction entre la configuration ROS et la SRAM de la carte.

        En Humble ce callback est appele AVANT que rclpy ne stocke la valeur
        (add_pre/post_set_parameters_callback n'arrivent qu'en Iron) : l'effet de bord doit
        donc etre produit ICI MEME, a partir de la liste recue, et non relu par get_parameter.

        DEUX PHASES, valider puis appliquer. Un lot partiellement rejete laisserait la carte
        dans un etat que plus aucun parametre ROS ne decrit.
        """
        new = {p.name: p.value for p in params}
        for name in self._POSITIVE_PARAMS:
            if name in new and float(new[name]) <= 0.0:
                return SetParametersResult(
                    successful=False, reason=f"{name} doit etre strictement positif")
        for name in self._PID_PARAMS:
            if name in new:
                arr = list(new[name])
                if len(arr) != 4:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{name} doit porter 4 gains (un par moteur M1..M4)")
                if any(float(v) < 0.0 for v in arr):
                    return SetParametersResult(
                        successful=False,
                        reason=f"{name} : un gain negatif inverserait la boucle")
        if "car_type" in new and not 1 <= int(round(float(new["car_type"]))) <= 6:
            return SetParametersResult(
                successful=False, reason="car_type hors 1..6 (4 = FOURWHEEL pour ce robot)")
        if "board_log_level" in new and not 0 <= int(new["board_log_level"]) <= 7:
            return SetParametersResult(
                successful=False, reason="board_log_level hors 0..7 (MAV_SEVERITY)")
        for name in ("encoder_left_index", "encoder_right_index"):
            if name in new and not 0 <= int(new[name]) <= 3:
                return SetParametersResult(
                    successful=False, reason=f"{name} hors 0..3 (voies encodeur M1..M4)")
        if "cmd_vel_timeout_s" in new and float(new["cmd_vel_timeout_s"]) <= 0.0:
            return SetParametersResult(
                successful=False,
                reason="cmd_vel_timeout_s doit etre > 0 : un watchdog nul ne freine jamais")
        if "board_readback_timeout_s" in new and float(new["board_readback_timeout_s"]) <= 0.0:
            return SetParametersResult(
                successful=False, reason="board_readback_timeout_s doit etre > 0")

        # --- application (toutes les valeurs du lot sont acceptables) ---
        if "counts_per_rev" in new:
            self.cpr = float(new["counts_per_rev"])
        if "wheel_diameter_m" in new:
            self.wheel_diameter = float(new["wheel_diameter_m"])
        if "wheel_separation_m" in new:
            self.wheel_sep = float(new["wheel_separation_m"])
        if "car_type" in new:
            self.car_type = float(new["car_type"])
        if "pid_kp" in new:
            self.pid_kp = [float(v) for v in new["pid_kp"]]
        if "pid_ki" in new:
            self.pid_ki = [float(v) for v in new["pid_ki"]]
        if "pid_kd" in new:
            self.pid_kd = [float(v) for v in new["pid_kd"]]
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
        if "board_log_level" in new:
            self.board_log_level = int(new["board_log_level"])
        if "board_readback_timeout_s" in new:
            self.readback_timeout = float(new["board_readback_timeout_s"])
        if "publish_odom_tf" in new:
            self.publish_odom_tf = bool(new["publish_odom_tf"])
            if self.publish_odom_tf and self.tf_broadcaster is None:
                self.tf_broadcaster = TransformBroadcaster(self)

        touch_geom = any(k in new for k in self._GEOM_PARAMS)
        touch_pid = any(k in new for k in self._PID_PARAMS)
        if touch_geom or touch_pid:
            # save=False : un reglage a chaud reste en SRAM. La flash, c'est save_board_config.
            self._push_board_config(geom=touch_geom, pid=touch_pid, save=False)
        # L'armement EN DERNIER : si une autre valeur du meme lot avait ete refusee, on n'aura
        # pas arme un robot sur une configuration a moitie appliquee.
        if "enable_cmd_vel" in new:
            self._apply_actuation(bool(new["enable_cmd_vel"]))
        return SetParametersResult(successful=True)

    # --- actuation moteur ---------------------------------------------------
    def _apply_actuation(self, enable):
        """Cree ou detruit la souscription /cmd_vel. Idempotent.

        C'est ce qui rend l'armement pilotable depuis MCP sans redemarrer le noeud, et c'est
        aussi ce qui rend le verrou VERIFIABLE : `ros2 topic info /cmd_vel` doit montrer ZERO
        souscripteur tant que enable_cmd_vel est faux. Un simple `if` dans le callback
        laisserait un souscripteur visible, donc un verrou qu'on ne peut pas constater.
        """
        self.enable_cmd_vel = bool(enable)
        if self.enable_cmd_vel and self.sub_cmd is None:
            self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
            self.get_logger().warn("ACTUATION MOTEUR ARMEE : /cmd_vel est souscrit.")
        elif not self.enable_cmd_vel and self.sub_cmd is not None:
            # Freiner AVANT de lacher la souscription : sinon une derniere consigne non nulle
            # resterait en vigueur sur la carte, et plus aucun topic ne permettrait de la
            # reprendre.
            self._brake()
            self.destroy_subscription(self.sub_cmd)
            self.sub_cmd = None
            self._last_cmd_t = None
            self.get_logger().info("actuation moteur desarmee : /cmd_vel n'est plus souscrit.")

    def _brake(self):
        """Arret moteur. Ceinture ET bretelles, volontairement.

        Cette carte implemente REELLEMENT l'arret (contrairement a l'ESP32, ou link.stop()
        emet un message dont le `case` est vide), donc un seul envoi suffirait. On emet malgre
        tout trois consignes nulles : une trame perdue sur un UART partage avec la telemetrie
        ne doit pas laisser le robot en mouvement.
        """
        for _ in range(3):
            self.link.sendCmdVel(0.0, 0.0)

    def _on_cmd_vel(self, msg):
        if not self.enable_cmd_vel:
            return
        self.link.sendCmdVel(float(msg.linear.x), float(msg.angular.z))
        self._last_cmd_t = self.get_clock().now()

    # --- boucle de publication ---------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        snap = self.link.snapshot()
        sp = self.link.encSpeed()
        cpr = self.link.cpr or self.cpr

        # Watchdog : on freine UNE fois, puis on oublie la consigne. Sans ce reset, un robot a
        # l'arret reemettrait une consigne nulle 30 fois par seconde pour rien, sur un UART
        # partage avec la telemetrie.
        if self.enable_cmd_vel and self._last_cmd_t is not None:
            if (now - self._last_cmd_t).nanoseconds * 1e-9 > self.cmd_vel_timeout:
                self._brake()
                self._last_cmd_t = None

        stamp = now.to_msg()

        # --- RPM par roue (4 voies REELLES sur cette carte) ---
        if sp:
            rpm = WheelRpm()
            rpm.header.stamp = stamp
            rpm.rpm = [s / cpr * 60.0 for s in sp]
            self.pub_rpm.publish(rpm)

        # --- reglage du PID : mesure et consigne telles que la boucle EMBARQUEE les voit ---
        # Publie SEULEMENT si la carte fournit les deux : mieux vaut un topic absent qu'un
        # topic de zeros qu'on prendrait pour une mesure.
        meas = snap.get("motor_rpm")
        req = snap.get("motor_rpm_req")
        counts = snap.get("encoders") or []
        if meas and req:
            js = JointState()
            js.header.stamp = stamp
            js.name = list(JOINT_NAMES)
            # position en RADIANS (convention respectee, robot_state_publisher reste juste),
            # velocity en RPM (ecart assume : comparaison directe aux releves de BambooV2).
            js.position = ([c / cpr * 2.0 * math.pi for c in counts[:4]]
                           if len(counts) >= 4 else [])
            js.velocity = [float(v) for v in meas[:4]]
            self.pub_joint.publish(js)
            jr = JointState()
            jr.header.stamp = stamp
            jr.name = list(JOINT_NAMES)
            jr.velocity = [float(v) for v in req[:4]]
            self.pub_joint_req.publish(jr)

        # --- odometrie integree (la carte ne calcule pas la pose) ---
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        v = w = 0.0
        if sp and len(sp) > max(self.left_idx, self.right_idx) and dt > 0.0:
            dist_per_tic = math.pi * self.wheel_diameter / cpr
            v_l = sp[self.left_idx] * dist_per_tic * (-1.0 if self.inv_left else 1.0)
            v_r = sp[self.right_idx] * dist_per_tic * (-1.0 if self.inv_right else 1.0)
            v = 0.5 * (v_l + v_r)
            w = (v_r - v_l) / self.wheel_sep if self.wheel_sep > 1e-6 else 0.0
            self.th += w * dt
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qx, qy, qz, qw = _quat_from_euler(0.0, 0.0, self.th)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.01
        odom.twist.covariance[0] = 0.001
        odom.twist.covariance[35] = 0.01
        self.pub_odom.publish(odom)

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

        # --- IMU : la carte envoie des DEGRES, ROS veut des radians ---
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        roll = math.radians(float(snap.get("roll") or 0.0))
        pitch = math.radians(float(snap.get("pitch") or 0.0))
        yaw = math.radians(float(snap.get("yaw") or 0.0))
        qx, qy, qz, qw = _quat_from_euler(roll, pitch, yaw)
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        # -1 en tete de covariance = "cette grandeur n'est pas fournie", convention ROS : la
        # carte ne remonte ni gyro ni accelerometre brut dans ce snapshot.
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(imu)

        # --- magnetometre : le MPU9250 en a un, d'ou un cap ABSOLU possible ---
        mag_v = snap.get("mag")
        if mag_v and len(mag_v) >= 3:
            mag = MagneticField()
            mag.header.stamp = stamp
            mag.header.frame_id = self.imu_frame
            # uT -> Tesla (unite SI de sensor_msgs/MagneticField).
            mag.magnetic_field.x = float(mag_v[0]) * 1e-6
            mag.magnetic_field.y = float(mag_v[1]) * 1e-6
            mag.magnetic_field.z = float(mag_v[2]) * 1e-6
            self.pub_mag.publish(mag)

        # --- batterie : seuils microcode 9,6 V bas / 13,0 V haut ---
        # NE JAMAIS alimenter entre 8,6 et 9,5 V : arret latchant en 2 s, reset obligatoire.
        batt = BatteryState()
        batt.header.stamp = stamp
        batt.voltage = float(snap.get("battery") or 0.0)
        batt.present = True
        self.pub_batt.publish(batt)

        # --- etat brut agrege (diagnostic MCP) ---
        st = Stm32Status()
        st.header.stamp = stamp
        st.battery_voltage = float(snap.get("battery") or 0.0)
        st.roll = float(snap.get("roll") or 0.0)
        st.pitch = float(snap.get("pitch") or 0.0)
        st.yaw = float(snap.get("yaw") or 0.0)
        st.vx = float(snap.get("vx") or 0.0)
        st.vy = float(snap.get("vy") or 0.0)
        st.vz = float(snap.get("vz") or 0.0)
        st.encoders = [int(c) for c in counts]
        # Bloc de relecture : ce que la carte dit avoir en SRAM, donc un ETAT, pas une mesure.
        cfg = self.board_cfg
        st.board_config_ok = cfg is not None
        st.board_cpr = float(cfg["cpr"]) if cfg else 0.0
        st.board_circ_mm = float(cfg["circ_mm"]) if cfg else 0.0
        st.board_apb_mm = float(cfg["apb_mm"]) if cfg else 0.0
        st.board_pid_kp = [float(x) for x in cfg["kp"]] if cfg else []
        st.board_pid_ki = [float(x) for x in cfg["ki"]] if cfg else []
        st.board_pid_kd = [float(x) for x in cfg["kd"]] if cfg else []
        # Ombre d'hote S1..S4 : NaN = voie jamais commandee depuis ce demarrage.
        st.servo_angle_deg = [float(a) for a in self._servo_shadow]
        self.pub_status.publish(st)

    def destroy_node(self):
        try:
            # Freiner AVANT de fermer le port, et seulement si on etait arme : envoyer une
            # consigne sur un robot desarme serait une actuation que personne n'a demandee.
            if self.enable_cmd_vel:
                self._brake()
            # Les servos ne sont PAS recentres a l'arret : un mouvement de camera pendant un
            # `docker compose stop` n'est demande par personne, et la nacelle garde une pose
            # sure par construction (bornes 8-172 deg).
            self.link.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32MavlinkDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
