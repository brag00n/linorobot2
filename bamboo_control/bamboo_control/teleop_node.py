#!/usr/bin/env python3
"""bamboo_teleop -- /joy -> modes du groupe videotracking et pas relatifs des axes servo.

PORTAGE INTEGRAL DES CONTROLES DU PROTO `robot_controlv3` : stick gauche = deplacement,
stick droit = camera, boutons = modes du groupe videotracking. Meme loi d'axe, memes noms
de parametres (`deadzone`, `expo`, `tele_lin`, `tele_ang`), pour qu'un essai au banc se
transpose sans retraduction. Les equivalents proto sont cites en regard de chaque bloc.

TRACTION : PRESENTE, MAIS DERRIERE DEUX VERROUS EN SERIE. Ce noeud publie /cmd_vel comme
le proto (poussee du stick = vitesse, portage de RobotMotorDrive.holdVelocityAnalog).
Deux verrous INDEPENDANTS empechent une roue de tourner par surprise :
  1. `motion_on`, etat LOCAL de ce noeud, a `false` au demarrage : desarme, RIEN n'est
     publie -- pas meme un zero. Le clic du stick gauche l'arme (touche O du proto).
  2. `enable_cmd_vel` du driver, a `false` sur ce robot (pack 12,6 V brut sur des moteurs
     7,4 V nominaux) : meme arme, le driver ignore la consigne.
La manette ne peut donc pas contourner le second : c'est voulu, et c'est ce qui rend
l'essai "au topic" possible sans toucher a l'alimentation.

RELACHER LE STICK REND LA MAIN, il ne tient pas un zero. Au front descendant on emet un
STOP FRANC (trois cmd_vel nuls, ce que la carte lit comme Motion_Stop) puis on SE TAIT.
Republier zero en permanence empecherait tout autre producteur (navigation, MCP) de
piloter : le proto evite exactement cela, et c'est pourquoi le silence est la position de
repos de ce noeud.

TROIS DIVERGENCES REELLES ENTRE CONSOMMATEURS, TROUVEES EN LISANT LEUR CODE -- elles
expliquent la seule regle non evidente de ce fichier :

  ON N'ENVOIE JAMAIS DE VALEUR VIDE. Le .msg autorise `value: ""` comme "bascule ou
  cycle", mais les consommateurs ne s'accordent pas dessus :
    * `overlay_node` traite le vide comme une BASCULE (overlay_node.cpp:190-208) ;
    * `facerecog_node` NE BASCULE PAS : sur `recognition` a valeur vide il pose
      mode_="recognition" et ne le coupe jamais (facerecog_node.cpp:182-188) -- la
      pastille du HUD basculerait donc pendant que la reconnaissance reste allumee ;
    * `tracking_node` REFUSE une valeur vide pour `predict`/`tracker`/`detector`
      (tracking_node.cpp:271-295 : "predict_mode inconnu : ''", ou un reconfigure() qui
      echoue et restaure l'ancienne config) -- un cycle a valeur vide ne changerait donc
      rien du tout.
  Ce noeud tient donc l'etat localement et envoie TOUJOURS une valeur ABSOLUE. Les trois
  consommateurs sont alors d'accord par construction, et la commande devient idempotente
  -- ce que le .msg recommande deja pour un client MCP.

  Et pour que cet etat local ne derive pas quand MCP ou un autre producteur agit, ce
  noeud SOUSCRIT a son propre topic de modes : tout ModeCmd vu, d'ou qu'il vienne, met a
  jour le modele. Sans cela, un appui apres une commande MCP repartirait de l'etat d'il y
  a dix minutes.

L'APPRENTISSAGE N'EST PAS UN MODE, C'EST UNE ACTION. `face_train_node` sert une action
`videotracking/train_faces` (batch de dizaines de secondes, annulable, un seul goal a la
fois) ; le ModeCmd `training` ne circule QUE dans l'autre sens, en `value: "done"`, pour
faire recharger la galerie. Un bouton qui publierait `training` ne lancerait donc rien.

LA LOI D'AXE EST CELLE DU PROTO, ET ELLE NE S'APPLIQUE QU'UNE FOIS. `_axis()` est le
portage ligne a ligne de RobotMain._axis : hors zone morte on RENORMALISE l'amplitude
utile, puis on courbe par une expo cubique (fin au centre, pleine echelle au bord). D'ou
une consequence non evidente : `joy_linux_node` doit avoir `deadzone: 0.0`, sinon le
noyau ecrase a zero la plage meme que la renormalisation sert a etaler -- la zone morte
serait appliquee DEUX FOIS et la marche qu'on veut supprimer resterait.
Les gachettes ont leur propre lecture (`_axisRaw`) : au repos elles valent -1, pas 0, et
les passer dans la zone morte les ferait lire "enfoncees" en permanence.

LE NUDGE EST UN SERVICE, PAS UN TOPIC, et cela dicte la cadence. `/servo/nudge` est un
`bamboo_interfaces/srv/Nudge` (requete/reponse) : l'appeler aux 25 Hz du stick empilerait
les futures. On integre donc la vitesse du stick, on n'appelle qu'a `nudge_rate_hz`, et
JAMAIS tant qu'un appel est en vol -- les pas non envoyes s'accumulent, donc rien n'est
perdu, seulement regroupe (ce que la liste d'axes de Nudge.srv permet : pan ET tilt dans
un seul appel, atomiquement).

LES MODES (lot QUALIBRATION, 2026-10-03). Les boutons ne portent plus une fonction fixe :
ils portent celle du MODE courant (conduite, tracking, calibration, navigation...) et du
GROUPE d'actions courant de ce mode. Le mode est tenu par bamboo_mode_manager (conventions
de system_modes, cf. mode_manager_node.py) : ce noeud le DEMANDE (croix gauche/droite ->
/bamboo/change_mode "__next__"/"__previous__") et le LIT sur /bamboo/mode -- il ne le
devine jamais. La croix haut/bas change de groupe. La carte des touches vit dans
joy_bamboo.yaml (`keymap.*`, cf. keymap_model.py) ; B et la croix restent GLOBAUX (STOP,
armement, modes) dans tous les modes, et une carte qui les reprendrait est refusee.
Ce qui est affiche (mode, groupe, touches, etat on/off) part sur /teleop/state, que le HUD
du flux augmente et l'OLED de l'ESP32 reprennent.

PERTE DE MANETTE = ARRET DE LA CAMERA. Sans /joy depuis `joy_timeout_s`, la vitesse est
remise a zero. Avec `autorepeat_rate: 25.0` cote joy_linux, un stick tenu republie en
continu : le silence signifie donc reellement "plus de manette", et pas "stick immobile".
"""
import random
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import Joy, JoyFeedback, JoyFeedbackArray

from bamboo_interfaces.action import Calibrate, TrainFaces
from bamboo_interfaces.msg import ModeCmd, NavState, RobotMode, TeleopState
from bamboo_interfaces.srv import ChangeMode, NavCommand, Nudge
from bamboo_control.keymap_model import KEYS, LONG, PREFIX, displayKey, parseKeymap

# Enumerations RESOLUES ICI (cf. docstring : le consommateur refuse une valeur vide).
# A garder alignees sur face_detection.cpp (detecteurs et trackers) et sur
# tracking_node.cpp (prediction).
_TRACKERS = ("none", "mil", "vit")
# Algorithmes de TRK-V (suivi video) : "none" n'en est pas un, c'est TRK-V COUPE.
_TRK_ALGOS = ("mil", "vit")
# Durees de capture CAPT (s), 0 = sans limite (seul le plafond par episode joue).
_CAPT_DURATIONS = (0, 10, 20, 30, 60)
_DETECTORS = ("haar", "dnn", "yunet")
_PREDICTS = ("off", "anticip", "coast")

# Carte de REPLI, si celle du fichier est refusee : la conduite et ses reglages de vitesse,
# rien d'autre. Le diagnostic `modes` passe alors en ERROR et dit pourquoi.
_FALLBACK_KEYMAP = {
    "conduite": {"groups": ["vitesse"], "bind": {"vitesse": {
        "ZR": "speed_max_up", "ZL": "speed_max_down",
        "PLUS": "speed_min_up", "MINUS": "speed_min_down"}}},
}


class TeleopNode(Node):
    """Traducteur /joy -> ModeCmd + Nudge. Aucune connaissance de carte, aucun moteur."""

    def __init__(self):
        super().__init__("bamboo_teleop")

        # --- index des commandes : faits d'HOTE, tous parametres --------------
        # Defauts = table MESUREE le 2026-09-30 sur la "GamepadX" en mode XInput
        # (essai T20 de bambooSTM32YB, cf. config/joy_bamboo.yaml, qui porte le releve
        # complet et les signes). Ils ne valent que pour CETTE manette dans CE mode :
        # le mode d'appairage change la table, et une passe ORDONNEE en avait produit
        # une fausse -- chaque controle a donc ete actionne DEUX FOIS.
        self._axPan = self.declare_parameter("axis_pan", 3).value
        self._axTilt = self.declare_parameter("axis_tilt", 4).value
        self._axDpadX = self.declare_parameter("axis_dpad_x", 6).value
        self._axDpadY = self.declare_parameter("axis_dpad_y", 7).value
        # Deplacement : stick gauche, et les deux gachettes analogiques.
        self._axDriveX = self.declare_parameter("axis_drive_x", 0).value
        self._axDriveY = self.declare_parameter("axis_drive_y", 1).value
        self._axTrigL = self.declare_parameter("axis_trigger_l", 2).value
        self._axTrigR = self.declare_parameter("axis_trigger_r", 5).value
        # SIGNE de TOUS les axes, applique a la LECTURE (_axis et _axisRaw), avant toute loi.
        # Releve T20 bis de BambooWS (2026-10-03) : la meme manette, meme mode XInput, rendait
        # les HUIT axes inverses par rapport au releve T20 du 2026-09-30, a index identiques --
        # sticks, gachettes et croix. Une seule valeur suffit donc, et elle remet aussi les
        # gachettes dans la convention de la loi (repos -1, appui +1). Ce qui a change entre
        # les deux releves n'est pas etabli : la manette est desormais LIEE (bonding).
        self._axesSign = -1.0 if float(self.declare_parameter("axes_sign", 1.0).value) < 0 \
            else 1.0
        # Face Nintendo numerotee a la Xbox : le B PHYSIQUE prend l'index 0, la ou on
        # attendrait A. Releve, pas deduit.
        # B (STOP au front, armement en appui long) est GLOBAL : il ne passe pas par la
        # carte des touches des modes, et aucune ne peut le reprendre.
        self._btn = {
            "stop": self.declare_parameter("button_stop", 0).value,
            "motion": self.declare_parameter("button_motion", 0).value,
        }
        # Index des TOUCHES des modes (releve en tete de joy_bamboo.yaml). ZL/ZR sont des
        # AXES (gachettes tout ou rien), lus par _axisRaw.
        self._keyIdx = {
            "A": self.declare_parameter("button_a", 1).value,
            "X": self.declare_parameter("button_x", 3).value,
            "Y": self.declare_parameter("button_y", 2).value,
            "L": self.declare_parameter("button_l", 4).value,
            "R": self.declare_parameter("button_r", 5).value,
            "MINUS": self.declare_parameter("button_minus", 6).value,
            "PLUS": self.declare_parameter("button_plus", 7).value,
        }
        # Cette manette ne DECLARE PAS de clic de stick (descripteur a 10 codes, sans
        # BTN_THUMBL/BTN_THUMBR) : l'armement passe donc en appui LONG sur B, qui garde son
        # STOP au front. Le partage est DEDUIT de l'egalite des index : sur une manette qui
        # declare ses clics de stick, des index distincts rendent les deux fronts
        # independants, sans toucher a ce fichier.
        self._longPress = max(0.1, float(
            self.declare_parameter("long_press_s", 0.6).value))
        self._sharedMotion = (self._btn["motion"] == self._btn["stop"])

        # --- axes de servo : des NOMS, jamais un numero de voie ---------------
        self._panName = self.declare_parameter("pan_axis_name", "pan").value
        self._tiltName = self.declare_parameter("tilt_axis_name", "tilt").value
        self._invPan = -1.0 if self.declare_parameter("invert_pan", False).value else 1.0
        self._invTilt = -1.0 if self.declare_parameter("invert_tilt", True).value else 1.0

        self._maxDegS = float(self.declare_parameter("nudge_max_deg_s", 45.0).value)
        self._nudgeHz = max(1.0, float(self.declare_parameter("nudge_rate_hz", 10.0).value))
        self._minDeg = float(self.declare_parameter("nudge_min_deg", 0.3).value)
        self._nudgeOn = bool(self.declare_parameter("nudge_enabled", True).value)
        # Le robot porte-t-il une NACELLE ? Pousse par control.launch.py depuis le fichier
        # canonique (presence de `servo_axes`). Sans nacelle (BambooWS), le chemin servo est
        # coupe une fois pour toutes : il n'y a rien a bouger, et appeler /servo/nudge
        # reviendrait a solliciter un service que personne ne sert.
        self._hasGimbal = bool(self.declare_parameter("has_gimbal", True).value)
        if not self._hasGimbal:
            self._nudgeOn = False
        self._debounce = float(self.declare_parameter("debounce_s", 0.25).value)
        # Loi d'axe du proto (RobotMain._axis) : memes noms, memes defauts.
        self._deadzone = min(0.9, max(0.0, float(
            self.declare_parameter("deadzone", 0.12).value)))
        self._expo = min(1.0, max(0.0, float(
            self.declare_parameter("expo", 0.35).value)))
        # Deplacement : plafonds du proto (TELE_LIN / TELE_ANG) et plage de niveaux.
        self._teleLin = float(self.declare_parameter("tele_lin", 0.5).value)
        self._teleAng = float(self.declare_parameter("tele_ang", 1.5).value)
        self._speedMin = int(self.declare_parameter("speed_min_level", 1).value)
        # Stick CARRE (essai au sol 2026-10-05) : la butee du stick est RONDE, donc a fond en
        # diagonale avant et rotation ne valent que ~0,7 chacun -- le robot RALENTISSAIT des
        # qu'on tournait stick a fond. On ramene le cercle au carre : diagonale = (1, 1).
        self._stickSquare = bool(self.declare_parameter("stick_square", True).value)
        # PLANCHER sur l'AMPLITUDE du stick, pas axe par axe : applique par axe, un soupcon
        # d'avant ajoute a une rotation faisait sauter la vitesse lineaire de 0 au plancher
        # (40 % au niveau 3) -- le "saut brutal" constate en passant de la rotation a plat a
        # l'arc. "axis" = l'ancienne loi du prototype.
        self._floorMode = str(self.declare_parameter("speed_floor_mode", "magnitude").value)
        self._speedMax = int(self.declare_parameter("speed_max_level", 3).value)
        self._cmdHz = max(1.0, float(self.declare_parameter("cmd_vel_rate_hz", 10.0).value))
        self._motionOn = bool(self.declare_parameter("motion_enabled", False).value)
        # VIBRATION de la manette a l'armement (1 impulsion) et au desarmement (2) : un retour
        # LOCAL, qui ne depend pas du Wi-Fi. Le HUD arrive par le reseau ; quand il est en
        # retard, l'operateur croit l'appui rate, rappuie et rebascule (constat du
        # 2026-10-10). joy_linux_node joue la vibration sur son peripherique `dev_ff`.
        # SANS EFFET sur la Under Control 2919 en XInput Bluetooth (essai du 2026-10-11) :
        # elle annonce FF_RUMBLE et le noyau accepte l'effet, mais elle ne vibre pas, meme
        # pilotee en direct sans ROS. Garde pour une manette qui le fait ; le retour qui
        # marche ici est l'asterisque de l'OLED (driver ESP32).
        self._ffPub = self.create_publisher(JoyFeedbackArray, "joy/set_feedback", 1)
        self._ffSeq = []              # [(instant monotone, intensite)], joue par _ffTick
        self.create_timer(0.05, self._ffTick)
        self._targetStep = float(self.declare_parameter("target_step", 0.01).value)
        self._joyTimeout = float(self.declare_parameter("joy_timeout_s", 0.5).value)
        # PERTE DE MANETTE (Bluetooth coupe, manette eteinte) : au-dela de ce silence de /joy,
        # la traction est DESARMEE -- pas seulement stoppee. Au retour de la manette il faut
        # un appui LONG sur B pour rearmer : un robot qui repartirait seul au retour du lien
        # sur un stick reste pousse est exactement ce que l'homme-mort interdit. Plus long que
        # joy_timeout_s (qui ne fait que stopper) pour ne pas desarmer sur un hoquet radio.
        self._disarmOnLoss = float(self.declare_parameter("disarm_on_loss_s", 1.0).value)
        # Miroir horizontal de l'image (action `flip_image`, ZR long) : parametre `flip` du
        # noeud de suivi, qui l'applique A LA TRAME avant detection.
        self._trackNode = str(self.declare_parameter("tracking_node", "/tracking_node").value)

        modeTopic = self.declare_parameter("mode_topic", "/videotracking/mode_cmd").value
        gamepadTopic = self.declare_parameter(
            "gamepad_state_topic", "/gamepad/connected").value
        nudgeSrv = self.declare_parameter("nudge_service", "/servo/nudge").value
        trainAction = self.declare_parameter(
            "train_action", "/videotracking/train_faces").value

        # --- modele local des modes (cf. docstring) ---------------------------
        self._trackingOn = False
        self._recog = "off"              # off / recognition / acquisition
        self._tracker = _TRACKERS[1]     # mil : le seul valide avant OpenCV 4.10
        self._detector = _DETECTORS[1]   # dnn : robuste de profil
        self._predict = _PREDICTS[0]
        # Chaine de TRACKING (cf. joy_bamboo.yaml) : ACQ -> TRK-V -> TRK-P, CAPT, GEN, RECO.
        self._acqOn = False             # detection (detect_enabled de tracking_node)
        # Suivi video COUPE au demarrage, comme la detection (tracking_node demarre en
        # track_mode `none`, videotracking.yaml) : CPU basse tant qu'on ne demande rien.
        self._trkvOn = False            # suivi video : tracker != none
        self._trkvAlgo = _TRK_ALGOS[0]
        self._captDuration = 0
        self._genOn = False             # apprentissage en cours (jusqu'au RESULTAT)
        # Numeros des ModeCmd partis d'une valeur ALEATOIRE : les consommateurs rejettent une
        # commande de meme `seq` que la precedente (garde anti-rejeu du message latche), TOUS
        # emetteurs confondus. Parti de 0, la manette relancee rejouait des numeros deja vus
        # (sa session d'avant, le gestionnaire de modes, l'apprentissage) : « tracker -> mil »
        # ignore apres un redemarrage (2026-10-13).
        self._seq = random.randint(1_000_000, 2_000_000_000)

        # --- etat d'entree ----------------------------------------------------
        self._prev = {}          # nom logique -> etat precedent (0/1)
        self._lastFire = {}      # nom logique -> horodatage du dernier declenchement
        self._hold = {}          # nom logique -> {t0, fired} de l'appui long en cours
        self._velPan = 0.0
        self._velTilt = 0.0
        self._pendPan = 0.0
        self._pendTilt = 0.0
        self._lastJoy = 0.0
        # Deplacement : consigne courante et memoire du front descendant du stick.
        self._lin = 0.0
        self._ang = 0.0
        self._gpDriving = False
        self._inFlight = False
        self._trainGoal = None
        self._warned = set()
        # Compteurs du diagnostic, remis a zero a chaque periode de l'updater.
        self._joyCount = 0
        self._nudgeRefusals = 0
        self._diagT = time.monotonic()

        # Latche : un noeud du groupe qui demarre APRES l'appui doit recevoir l'etat
        # demande. QoS identique cote consommateurs (profondeur 1, reliable).
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._modePub = self.create_publisher(ModeCmd, modeTopic, latched)
        # ETAT DE LA MANETTE pour la pastille MANETTE du HUD -- portage de la metrique
        # `gamepad_input` du prototype et de son `connected`. sensor_msgs/Joy n'a pas de
        # champ "connectee", et un /joy silencieux ne dit pas QUI manque : la manette, ou ce
        # noeud. D'ou un message a part, publie EN CONTINU (2 Hz) : c'est son SILENCE qui
        # dit au HUD que ce noeud ne tourne pas (pastille grise), et sa valeur qui dit si la
        # manette repond (verte / rouge). Le verdict s'appuie sur `autorepeat_rate` de
        # joy_linux_node, qui republie un stick TENU : ne plus rien recevoir ne peut donc
        # pas vouloir dire "manette immobile", seulement "manette absente".
        self._gamepadPub = self.create_publisher(Bool, gamepadTopic, 1)
        self._gamepadTimer = self.create_timer(0.5, self._onGamepadTick)
        self._modeSub = self.create_subscription(
            ModeCmd, modeTopic, self._onModeEcho, latched)

        # Nom LOGIQUE : c'est `control.launch.py` qui le remappe (argument
        # `cmd_vel_out_topic`). Un remappage sans clef correspondante est ignore EN
        # SILENCE, d'ou l'exposition explicite de celle-ci.
        self._velPub = self.create_publisher(Twist, "cmd_vel", 10)

        self._joySub = self.create_subscription(Joy, "joy", self._onJoy, 10)
        self._nudgeCli = self.create_client(Nudge, nudgeSrv)
        self._trainCli = ActionClient(self, TrainFaces, trainAction)

        # --- calibration (bamboo_calibration) ----------------------------------
        calNode = str(self.declare_parameter("calibration_node", "/bamboo_calibration").value)
        self._calCli = ActionClient(self, Calibrate, calNode + "/calibrate")
        self._calApplyCli = self.create_client(Trigger, calNode + "/apply_candidate")
        self._calRevertCli = self.create_client(Trigger, calNode + "/revert")
        self._calPiCli = self.create_client(SetBool, calNode + "/set_pi")
        self._calVariants = [v.strip() for v in str(self.declare_parameter(
            "calibration_variants", "essai").value).split(";") if v.strip()] or ["essai"]
        self._calVariant = self._calVariants[0]
        self._calArcs = bool(self.declare_parameter("calibration_include_arcs", True).value)
        self._calGoal = None          # future d'envoi, puis but en cours
        self._calHandle = None
        self._calText = ""
        self._calApplied = False
        self._calPi = False
        self._flipOn = None           # inconnu tant que tracking_node n'a pas ete interroge
        # --- navigation (bamboo_navigation) : une seule porte, ~/command du superviseur ---
        self._navCli = self.create_client(NavCommand, str(self.declare_parameter(
            "nav_command_service", "/nav_supervisor/command").value))
        self._navState = None
        self._navStateT = 0.0
        # PILE NAV lancee / arretee a la manette (A long, 2026-10-13) : l'utilisateur la VEUT
        # (True / False), la commande de sous-mode est-elle partie, et l'arret en cours.
        self._navWant = False
        self._navStartSent = False
        self._navStopT = None
        self._navWantT, self._navRetryAt, self._navSentT = 0.0, 0.0, 0.0
        self._navStarted = False      # allumee au moins une fois depuis le dernier A long
        self._navOffSince = None      # retombee a l'arret depuis (grace, cf. _navStackTick)
        self._navStackCli = self.create_client(SetBool, "/bamboo/nav_stack")
        self.create_timer(1.0, self._navStackTick)
        self._curAction = None        # action en cours d'execution (cf. _run)
        self._pendingActs = set()     # actions dont la commande attend sa reponse
        self._confirm = None          # confirmation en cours (cf. _askConfirm)
        self._confirmIgnore = set()   # touches qui y ont repondu, ignorees jusqu'au relache
        self.create_subscription(NavState, "/navigation/state",
                                 self._onNavState,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._flipGet = self.create_client(GetParameters, self._trackNode + "/get_parameters")
        self._flipSet = self.create_client(SetParameters, self._trackNode + "/set_parameters")

        # --- modes du robot (bamboo_mode_manager) ------------------------------
        # Sans gestionnaire, on reste dans `fallback_mode` : la conduite marche toujours.
        self._mode = str(self.declare_parameter("fallback_mode", "conduite").value)
        self._modeLabel = self._mode.upper()
        self._group = 0
        self._modeCli = self.create_client(
            ChangeMode, self.declare_parameter("change_mode_service",
                                               "/bamboo/change_mode").value)
        self._modeInFlight = False
        self._keymapError = ""
        self._actions = self._buildActions()
        self._keymap = self._loadKeymap()
        # Etat AFFICHE (HUD, OLED) : latche, a chaque changement et au moins a 1 Hz.
        self._statePub = self.create_publisher(
            TeleopState, self.declare_parameter("teleop_state_topic", "/teleop/state").value,
            latched)
        self._stateTimer = self.create_timer(1.0, self._publishState)
        # Etat de DEPART du suivi video annonce aux autres (HUD : pastille TRK-V), qui
        # supposaient sinon un tracker `mil` actif (2026-10-13). Une fois, un peu apres le
        # demarrage : tracking_node est parti en track_mode `none` (videotracking.yaml).
        self._startTimer = self.create_timer(3.0, self._announceStart)
        self._modeStateSub = self.create_subscription(
            RobotMode, self.declare_parameter("robot_mode_topic", "/bamboo/mode").value,
            self._onRobotMode, latched)

        self._timer = self.create_timer(1.0 / self._nudgeHz, self._onNudgeTick)
        self._velTimer = self.create_timer(1.0 / self._cmdHz, self._onDriveTick)
        # `nudge_enabled` est un COUPE-CIRCUIT de securite : il doit obeir A CHAUD. Sans
        # ce rappel il n'etait lu qu'A LA CONSTRUCTION, donc `ros2 param set` repondait
        # "Set parameter successful" et ne changeait RIEN -- un succes MENSONGER, et c'est
        # le pire defaut possible pour un interrupteur de securite : on croit avoir coupe.
        self.add_on_set_parameters_callback(self._onParamSet)

        # Diagnostics (lot DIAGNOSTICS) : les 4 taches du composant bamboo_teleop du module
        # bamboo_control. Les couches SOUS ce noeud (adaptateur Bluetooth, js0) sont
        # observees de l'exterieur par bamboo_health, qui ne peut pas ouvrir js0 sans devenir
        # un second lecteur du peripherique de joy_linux_node.
        # 20 Hz : joy_linux_node republie a 25 Hz (`autorepeat_rate`) un stick TENU.
        self._joyRateWarn = float(self.declare_parameter("joy_rate_warn_hz", 20.0).value)
        self._diag = Updater(self)
        self._diag.setHardwareID("bamboo_control")
        self._diag.add("manette", self._diagGamepad)
        self._diag.add("chemin servo", self._diagServo)
        self._diag.add("traction", self._diagTraction)
        self._diag.add("modes", self._diagModes)
        self.get_logger().info(
            "bamboo_teleop pret : modes -> %s, nudge -> %s (%.0f deg/s a fond, %.0f Hz), "
            "apprentissage -> %s, deplacement -> %s a %.0f Hz (moteurs %s au demarrage)."
            % (modeTopic, nudgeSrv, self._maxDegS, self._nudgeHz, trainAction,
               self._velPub.topic_name, self._cmdHz,
               "ARMES" if self._motionOn else "desarmes"))

    def _rumble(self, pulses, on_s=0.2, off_s=0.15):
        """`pulses` impulsions de vibration ; remplace une sequence en cours."""
        t = time.monotonic()
        seq = []
        for k in range(pulses):
            seq += [(t + k * (on_s + off_s), 1.0), (t + k * (on_s + off_s) + on_s, 0.0)]
        self._ffSeq = seq

    def _ffTick(self):
        now = time.monotonic()
        while self._ffSeq and self._ffSeq[0][0] <= now:
            _, v = self._ffSeq.pop(0)
            # id 0 = moteur fort, id 1 = moteur faible (convention de joy_linux).
            self._ffPub.publish(JoyFeedbackArray(array=[
                JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=i, intensity=float(v))
                for i in (0, 1)]))

    def _onParamSet(self, params):
        """Prise a chaud des SEULS parametres qu'il est sur de changer en marche.

        On ne generalise deliberement pas a tous les parametres : les index d'axes et de
        boutons decrivent un MATERIEL (cf. le releve en tete de joy_bamboo.yaml) et les
        changer en marche ferait repondre la manette autrement sans que rien ne le dise.
        """
        for p in params:
            if p.name == "nudge_enabled":
                # Sans nacelle, rien ne peut le rearmer : il n'y a pas de servo a commander.
                self._nudgeOn = bool(p.value) and self._hasGimbal
                # On JETTE les pas deja accumules. Sans cela, couper le chemin laisserait
                # partir un dernier nudge au reveil, sur une intention perimee -- exactement
                # ce que la perte de manette prend soin d'eviter plus bas.
                self._pendPan = self._pendTilt = 0.0
                self.get_logger().info(
                    "chemin servo %s a chaud" % ("ARME" if self._nudgeOn else "COUPE"))
        return SetParametersResult(successful=True)

    def _onGamepadTick(self):
        """Publie l'etat de la manette : vraie si un /joy est arrive dans `joy_timeout_s`.

        MEME seuil que la perte de manette du chemin servo et de la traction : la pastille
        ne peut donc pas montrer "connectee" pendant que le noeud, lui, la juge perdue.
        """
        msg = Bool()
        msg.data = self._gamepadConnected()
        self._gamepadPub.publish(msg)

    def _gamepadConnected(self):
        """LE verdict de manette connectee, et il n'y en a qu'un.

        Le lisent la pastille MANETTE (/gamepad/connected) et le diagnostic : une seule
        expression, donc les deux ne peuvent pas diverger.
        """
        return (time.monotonic() - self._lastJoy) <= self._joyTimeout

    # ------------------------------------------------------------- diagnostics
    def _diagPeriod(self):
        """Duree reelle depuis le dernier passage, pour des cadences justes meme si la
        periode de l'updater a glisse."""
        now = time.monotonic()
        dt, self._diagT = max(1e-3, now - self._diagT), now
        return dt

    def _diagGamepad(self, stat):
        hz = self._joyCount / self._diagPeriod()
        self._joyCount = 0
        stat.add("joy_hz", "%.1f" % hz)
        stat.add("seuil_warn_hz", "%.0f" % self._joyRateWarn)
        stat.add("joy_timeout_s", "%.2f" % self._joyTimeout)
        if not self._gamepadConnected():
            # OK et non WARN : l'absence est DEJA signalee, plus pres de sa cause, par le
            # composant `js0: peripherique` de bamboo_health (manette eteinte ou hors de
            # portee). Deux WARN pour un seul fait faisaient compter double la meme alerte
            # (constate 2026-10-04, mode SANTE). Cette tache garde ce qui n'appartient qu'a
            # elle : la cadence de /joy quand la manette EST la.
            stat.summary(DiagnosticStatus.OK, "manette absente : voir js0 (traction desarmee)")
        elif hz < self._joyRateWarn:
            stat.summary(DiagnosticStatus.WARN, "/joy lent : %.1f Hz" % hz)
        else:
            stat.summary(DiagnosticStatus.OK, "manette connectee, /joy %.0f Hz" % hz)
        return stat

    def _diagServo(self, stat):
        refusals, self._nudgeRefusals = self._nudgeRefusals, 0
        ready = self._nudgeCli.service_is_ready()
        stat.add("nudge_enabled", str(self._nudgeOn))
        stat.add("serveur_pret", str(ready))
        stat.add("service", self._nudgeCli.srv_name)
        stat.add("refus_periode", str(refusals))
        stat.add("nacelle", str(self._hasGimbal))
        if not self._hasGimbal:
            # OK et non WARN : l'absence de nacelle est l'etat NORMAL de ce robot.
            stat.summary(DiagnosticStatus.OK, "pas de nacelle sur ce robot (stick droit inactif)")
        elif not self._nudgeOn:
            stat.summary(DiagnosticStatus.WARN, "chemin servo COUPE (nudge_enabled=false)")
        elif not ready:
            stat.summary(DiagnosticStatus.WARN,
                         "aucun serveur %s : groupe suivi arrete ?" % self._nudgeCli.srv_name)
        elif refusals:
            stat.summary(DiagnosticStatus.ERROR,
                         "%d nudge(s) refuse(s) par le serveur sur la periode" % refusals)
        else:
            stat.summary(DiagnosticStatus.OK, "arme, serveur pret")
        return stat

    def _diagTraction(self, stat):
        subs = self._velPub.get_subscription_count()
        stat.add("motion_enabled", str(self._motionOn))
        stat.add("abonnes_cmd_vel", str(subs))
        stat.add("plancher_niveau", str(self._speedMin))
        stat.add("plafond_niveau", str(self._speedMax))
        if not self._motionOn:
            stat.summary(DiagnosticStatus.OK, "desarmee (etat nominal)")
        elif subs == 0:
            # Arme mais sans effet : le driver ne s'abonne a cmd_vel que si enable_cmd_vel est
            # vrai, ce qui n'est pas le cas sur ce robot. A voir, pas une panne.
            stat.summary(DiagnosticStatus.WARN,
                         "ARMEE mais sans abonne a %s (enable_cmd_vel=false ?)"
                         % self._velPub.topic_name)
        else:
            stat.summary(DiagnosticStatus.OK, "ARMEE, %d abonne(s)" % subs)
        return stat

    def _diagModes(self, stat):
        subs = self._modePub.get_subscription_count()
        train = self._trainCli.server_is_ready()
        stat.add("abonnes_modes", str(subs))
        stat.add("dernier_seq", str(self._seq))
        stat.add("apprentissage_pret", str(train))
        stat.add("mode_robot", self._mode)
        stat.add("groupe", self._groupName() or "-")
        stat.add("gestionnaire_modes", str(self._modeCli.service_is_ready()))
        if self._keymapError:
            stat.summary(DiagnosticStatus.ERROR,
                         "carte des touches REFUSEE (repli conduite) : %s" % self._keymapError)
        elif subs == 0:
            stat.summary(DiagnosticStatus.WARN,
                         "aucun abonne a %s : groupe suivi arrete ?" % self._modePub.topic_name)
        elif not train:
            stat.summary(DiagnosticStatus.WARN, "action d'apprentissage absente")
        else:
            stat.summary(DiagnosticStatus.OK, "%d abonne(s), apprentissage pret" % subs)
        return stat

    # ---------------------------------------------------------------- entrees
    def _axisRaw(self, msg, idx):
        """Lecture BRUTE, pour les gachettes.

        Defaut -1.0 et non 0.0 : une gachette au repos vaut -1, et l'absence d'axe doit
        donc se lire "relachee". Ne passe PAS par la loi d'axe (cf. en-tete). Le signe
        `axes_sign` s'applique a la valeur lue, PAS au defaut : il ramene une manette
        inversee dans la convention, il ne doit pas fabriquer une gachette enfoncee.
        """
        if 0 <= idx < len(msg.axes):
            return self._axesSign * float(msg.axes[idx])
        return -1.0

    def _raw(self, msg, idx):
        """Valeur BRUTE signee d'un axe (axes_sign applique), 0 si l'axe manque."""
        return self._axesSign * float(msg.axes[idx]) if 0 <= idx < len(msg.axes) else 0.0

    def _axis(self, msg, idx):
        """Loi d'axe du proto (RobotMain._axis), transposee telle quelle.

        Lecture TOLERANTE : une manette plus pauvre que la table ne fait pas crasher.
        """
        return self._law(self._raw(msg, idx))

    @staticmethod
    def _square(x, y):
        """Cercle -> carre : (0,707 ; 0,707) devient (1 ; 1), un axe pur reste inchange."""
        m = max(abs(x), abs(y))
        if m <= 1e-6:
            return 0.0, 0.0
        k = min(1.0, (x * x + y * y) ** 0.5) / m
        return max(-1.0, min(1.0, x * k)), max(-1.0, min(1.0, y * k))

    def _law(self, v):
        """Zone morte renormalisee + expo cubique sur UNE valeur (cf. _axis)."""
        m = abs(v)
        if m <= self._deadzone:
            return 0.0
        # Renormalisation : la plage utile [dz, 1] est etalee sur [0, 1], donc la sortie
        # repart de ZERO au franchissement et non de dz -- c est ce qui supprime la marche.
        s = (m - self._deadzone) / (1.0 - self._deadzone)
        # Expo cubique : fin autour du centre, pleine echelle conservee au bord.
        s = (1.0 - self._expo) * s + self._expo * s ** 3
        return s if v > 0.0 else -s

    @staticmethod
    def _button(msg, idx):
        return int(msg.buttons[idx]) if 0 <= idx < len(msg.buttons) else 0

    def _edge(self, name, value, now):
        """Front montant + anti-rebond.

        L'anti-rebond n'est pas cosmetique pour la CROIX : joy_linux la donne en AXE, un
        appui bref produit donc plusieurs echantillons a 25 Hz et sans garde on lancerait
        trois batchs d'apprentissage pour un seul appui.
        """
        prev = self._prev.get(name, 0)
        self._prev[name] = value
        if not (value and not prev):
            return False
        if now - self._lastFire.get(name, 0.0) < self._debounce:
            return False
        self._lastFire[name] = now
        return True

    def _holdEdge(self, name, value, now):
        """Discrimine appui COURT et appui LONG sur un MEME bouton.

        Retourne "long" des que le seuil est franchi, bouton TOUJOURS enfonce -- l action
        longue part donc sous le doigt, sans attendre le relachement. Retourne "short" au
        RELACHEMENT seulement, et c est une contrainte, pas un choix : on ne peut pas
        savoir qu un appui sera court avant qu il ne finisse. D ou la regle du YAML -- une
        action qui ne doit JAMAIS etre retardee (le STOP) reste sur le front montant et
        n emprunte pas ce chemin.
        """
        st = self._hold.setdefault(name, {"t0": None, "fired": False})
        if value:
            if st["t0"] is None:
                st["t0"] = now
            elif not st["fired"] and now - st["t0"] >= self._longPress:
                st["fired"] = True
                return "long"
            return None
        if st["t0"] is None:
            return None
        court = not st["fired"]
        st["t0"] = None
        st["fired"] = False
        if not court:
            return None
        # ANTI-REBOND, comme _edge : un bouton qui rebondit au relachement donnait deux
        # appuis courts a 0,1 s (2026-10-13 : deux changements de carte pour un appui sur +).
        if now - self._lastFire.get(name, 0.0) < self._debounce:
            return None
        self._lastFire[name] = now
        return "short"

    def _onJoy(self, msg):
        self._joyCount += 1
        now = time.monotonic()
        self._lastJoy = now

        # --- vitesse des axes servo (integree par le timer) ------------------
        self._velPan = self._invPan * self._axis(msg, self._axPan) * self._maxDegS
        self._velTilt = self._invTilt * self._axis(msg, self._axTilt) * self._maxDegS

        # --- deplacement : stick gauche --------------------------------------
        self._driveFromJoy(msg)

        if self._edge("stop", self._button(msg, self._btn["stop"]), now):
            # STOP inconditionnel : il vaut meme moteurs DESARMES, parce qu un zero franc
            # est la seule chose qu on veut pouvoir emettre sans reflechir. Il ANNULE aussi
            # une calibration en cours : le robot y tourne tout seul.
            self._calibCancel("STOP")
            self._stopMotion()
            if self._confirm is not None:
                self._confirmEnd("annulee (B)")
            if self._mode == "navigation" and self._navCli.service_is_ready():
                # B en NAV = STOP + annulation du but, de l'exploration auto et de la ronde
                # (2026-10-13) : sans cela Nav2 reprenait 0,5 s apres le STOP.
                self._nav("cancel_all")

        # Armement. Sur cette manette le bouton est le MEME que le STOP (pas de clic de
        # stick a lui donner) : le STOP garde son front montant juste au-dessus et
        # l armement s AJOUTE en appui long. Consequence assumee : armer emet un zero au
        # passage -- inoffensif moteurs desarmes -- et un geste TENU est une vertu pour le
        # verrou de la traction.
        armer = (self._holdEdge("motion", self._button(msg, self._btn["motion"]),
                                now) == "long") if self._sharedMotion else \
            self._edge("motion", self._button(msg, self._btn["motion"]), now)
        if armer:
            self._motionOn = not self._motionOn
            if not self._motionOn:
                self._calibCancel("desarmement")
                self._stopMotion()               # desarmer ARRETE, il ne fige pas
            self._publishState()
            self._rumble(1 if self._motionOn else 2)
            self.get_logger().warning(
                "moteurs %s a la manette (le verrou `enable_cmd_vel` du driver reste "
                "maitre)" % ("ARMES" if self._motionOn else "desarmes"))

        # --- croix : GLOBALE, dans tous les modes -----------------------------
        # SIGNE RELEVE, et il est CONTRE-INTUITIF : sur l'axe 7 le HAUT vaut -1 et le BAS
        # +1 (convention du noyau, comme pour l'axe vertical d'un stick).
        dx = self._axis(msg, self._axDpadX)
        dy = self._axis(msg, self._axDpadY)
        if self._edge("dpad_right", 1 if dx > 0.5 else 0, now):
            self._requestMode("__next__")
        if self._edge("dpad_left", 1 if dx < -0.5 else 0, now):
            self._requestMode("__previous__")
        if self._edge("dpad_up", 1 if dy < -0.5 else 0, now):
            self._cycleGroup(-1)
        if self._edge("dpad_down", 1 if dy > 0.5 else 0, now):
            self._cycleGroup(+1)

        # --- touches du MODE et du GROUPE courants ---------------------------
        # Une touche qui a un appui LONG dans ce groupe passe par _holdEdge (son appui court
        # part alors au RELACHEMENT) ; sinon elle part au front, comme le prototype.
        if self._confirm is not None:
            self._confirmJoy(msg, now)
            return
        bind = self._binding()
        for key in KEYS:
            value = self._keyValue(msg, key)
            tag = "key:" + key
            if key in self._confirmIgnore:
                # A repondu a une confirmation : ignoree jusqu'a son relachement.
                self._prev[tag] = value
                self._hold.pop(tag, None)
                if not value:
                    self._confirmIgnore.discard(key)
                continue
            short, long_ = bind.get(key), bind.get(key + LONG)
            if long_ is not None:
                ev = self._holdEdge(tag, value, now)
                if ev == "long":
                    self._run(long_)
                elif ev == "short" and short is not None:
                    self._run(short)
            elif short is not None:
                if self._edge(tag, value, now):
                    self._run(short)
            else:
                # Garder le front a jour : une touche liee plus tard (changement de groupe)
                # ne doit pas partir sur un appui deja en cours.
                self._prev[tag] = value

    # ----------------------------------------------------------- deplacement
    @staticmethod
    def _levelFrac(level):
        """Niveau 0..9 -> fraction (niveau+1)/10, comme RobotMotorDrive._levelFrac."""
        return (max(0, min(9, int(level))) + 1) / 10.0

    @staticmethod
    def _scaleAxis(v, fmin, fmax):
        """Poussee -> fraction entre plancher et plafond ; poussee nulle -> ZERO.

        Le plancher ne rampe donc pas au repos : il sert a DECOLLER malgre la friction
        des motoreducteurs, pas a garantir une vitesse minimale permanente.
        """
        if v == 0.0:
            return 0.0
        f = fmin + (fmax - fmin) * min(1.0, abs(v))
        return f if v > 0.0 else -f

    def _holdVelocityAnalog(self, fwd, turn):
        """Portage de RobotMotorDrive.holdVelocityAnalog : memes bornes, memes plafonds."""
        fmin = self._levelFrac(self._speedMin)
        fmax = self._levelFrac(self._speedMax)
        if self._floorMode == "axis":
            self._lin = self._scaleAxis(fwd, fmin, fmax) * self._teleLin
            self._ang = self._scaleAxis(turn, fmin, fmax) * self._teleAng
            return
        # Plancher sur l'AMPLITUDE : r = la plus grande des deux deviations. Le plancher porte
        # sur r, et chaque axe en garde sa PART (fwd/r, turn/r) : un axe pur se comporte
        # exactement comme avant, et un axe secondaire faible reste faible -- plus de saut.
        r = max(abs(fwd), abs(turn))
        if r <= 0.0:
            self._lin = self._ang = 0.0
            return
        f = fmin + (fmax - fmin) * min(1.0, r)
        self._lin = (fwd / r) * f * self._teleLin
        self._ang = (turn / r) * f * self._teleAng

    def _bumpSpeedMin(self, delta):
        self._speedMin = max(0, min(9, self._speedMin + int(delta)))
        if self._speedMin > self._speedMax:      # invariant min <= max
            self._speedMax = self._speedMin
        self._logSpeed("plancher")

    def _bumpSpeedMax(self, delta):
        self._speedMax = max(0, min(9, self._speedMax + int(delta)))
        if self._speedMax < self._speedMin:
            self._speedMin = self._speedMax
        self._logSpeed("plafond")

    def _logSpeed(self, which):
        self.get_logger().info(
            "vitesse (%s) : niveaux %d..%d -> %.2f..%.2f m/s"
            % (which, self._speedMin, self._speedMax,
               self._levelFrac(self._speedMin) * self._teleLin,
               self._levelFrac(self._speedMax) * self._teleLin))

    def _publishTwist(self):
        t = Twist()
        t.linear.x = self._lin
        t.angular.z = self._ang
        self._velPub.publish(t)

    def _stopMotion(self):
        """STOP FRANC : trois consignes nulles, ce que la carte lit comme Motion_Stop.

        Puis on SE TAIT -- on rend la main a un autre producteur au lieu de tenir un zero.
        """
        self._lin = 0.0
        self._ang = 0.0
        for _ in range(3):
            self._publishTwist()
        self._gpDriving = False

    def _driveFromJoy(self, msg):
        """Portage de RobotMain._drive_from_gamepad, front descendant compris."""
        fwd = turn = 0.0
        if self._motionOn:
            # avant = stick vers le HAUT (LY < 0) ; gauche = + (angular.z anti-horaire)
            ry = -self._raw(msg, self._axDriveY)
            rx = -self._raw(msg, self._axDriveX)
            if self._stickSquare:
                rx, ry = self._square(rx, ry)      # AVANT la zone morte : elle reste ronde
            fwd, turn = self._law(ry), self._law(rx)
        driving = (fwd != 0.0 or turn != 0.0)
        if driving and self._calBusy():
            # La manette REPREND LA MAIN : pousser le stick pendant une calibration l'annule.
            self._calibCancel("stick gauche")
        if driving:
            self._holdVelocityAnalog(fwd, turn)
            if not self._gpDriving:                  # front montant : demarrage franc
                self._publishTwist()
            self._gpDriving = True
        elif self._gpDriving:                        # front descendant : stop puis silence
            self._stopMotion()

    def _onDriveTick(self):
        """Republie la consigne tant que le stick est pousse ; l absence de manette arrete.

        Le silence de /joy vaut perte de manette : `autorepeat_rate` republie un stick
        TENU, donc ne plus rien recevoir ne peut pas vouloir dire "stick immobile".
        """
        silence = time.monotonic() - self._lastJoy
        if self._motionOn and self._lastJoy > 0.0 and silence > self._disarmOnLoss:
            # Manette PERDUE : desarmement, arret, annulation d'une calibration. Le rearmement
            # exige un appui LONG sur B, une fois la manette revenue.
            self._motionOn = False
            self._calibCancel("manette perdue")
            self._stopMotion()
            self.get_logger().error(
                "manette PERDUE depuis %.1f s : traction DESARMEE (B long pour rearmer)"
                % silence)
            self._publishState()
        if silence > self._joyTimeout:
            if self._gpDriving:
                self._warnOnce("joy_lost_drive",
                               "manette perdue en pleine poussee : STOP franc")
                self._stopMotion()
            return
        if self._gpDriving:
            self._publishTwist()

    @staticmethod
    def _next(values, current):
        try:
            return values[(values.index(current) + 1) % len(values)]
        except ValueError:
            return values[0]

    # ---------------------------------------------------------------- sorties
    def _announceStart(self):
        self._startTimer.cancel()
        if not self._trkvOn:
            self._sendMode("tracker", "none")

    def _sendMode(self, target, value):
        self._seq += 1
        m = ModeCmd()
        m.header.stamp = self.get_clock().now().to_msg()
        m.target = target
        m.value = value
        m.seq = self._seq
        self._modePub.publish(m)
        self.get_logger().info("mode %s -> %s (seq %d)" % (target, value, self._seq))

    def _onModeEcho(self, m):
        """Resynchronise le modele local sur TOUT producteur (MCP, un autre outil).

        Y compris nos propres messages, ce qui est inoffensif : on y remet la valeur
        qu'on vient de poser. Une valeur vide venue d'ailleurs est ignoree -- on ne sait
        pas ce que le consommateur en a fait (cf. docstring), donc on ne devine pas.
        Les noms `track_mode`/`predict_mode` sont acceptes en plus des noms du contrat :
        c'est ce que `tracking_node.cpp` ecoutait historiquement.
        """
        if not m.value:
            return
        if m.target == "tracking":
            self._trackingOn = m.value in ("on", "true", "1")
        elif m.target in ("recognition", "acquisition"):
            self._recog = "off" if m.value == "off" else m.value
        elif m.target in ("tracker", "track_mode") and m.value in _TRACKERS:
            self._tracker = m.value
            self._trkvOn = m.value != "none"
            if self._trkvOn:
                self._trkvAlgo = m.value
        elif m.target == "detect" and m.value in ("on", "off"):
            self._acqOn = m.value == "on"
        elif m.target == "detector" and m.value in _DETECTORS:
            self._detector = m.value
        elif m.target in ("predict", "predict_mode") and m.value in _PREDICTS:
            self._predict = m.value

    def _onNudgeTick(self):
        """Integre la vitesse du stick et envoie au plus UN appel par periode."""
        dt = 1.0 / self._nudgeHz
        if time.monotonic() - self._lastJoy > self._joyTimeout:
            # Manette perdue : on relache, et on JETTE les pas en attente. Les rejouer a
            # la reconnexion ferait bouger la camera sur une intention vieille de
            # plusieurs secondes.
            self._velPan = self._velTilt = 0.0
            self._pendPan = self._pendTilt = 0.0
            return
        if not self._nudgeOn:
            return

        self._pendPan += self._velPan * dt
        self._pendTilt += self._velTilt * dt
        if max(abs(self._pendPan), abs(self._pendTilt)) < self._minDeg:
            return
        if self._inFlight:
            # On n'abandonne pas le pas : il reste accumule pour la periode suivante.
            return
        if not self._nudgeCli.service_is_ready():
            self._warnOnce(
                "nudge_absent",
                "aucun serveur sur %s : le groupe tracking ne tourne pas (servocam_node en "
                "est proprietaire), et le driver ne sert ce service que si "
                "enable_nudge_service est arme -- il est FALSE par defaut, un seul serveur "
                "etant admis. Les pas de stick sont accumules, pas perdus."
                % self._nudgeCli.srv_name)
            return

        req = Nudge.Request()
        req.axis = [self._panName, self._tiltName]
        req.delta = [self._pendPan, self._pendTilt]
        self._pendPan = self._pendTilt = 0.0
        self._inFlight = True
        self._nudgeCli.call_async(req).add_done_callback(self._onNudgeDone)

    def _onNudgeDone(self, future):
        self._inFlight = False
        try:
            res = future.result()
        except Exception as exc:                                    # noqa: BLE001
            self.get_logger().warn("appel de nudge echoue : %s" % exc)
            self._nudgeRefusals += 1
            return
        if not res.success:
            self._nudgeRefusals += 1
            # Un axe inconnu est une erreur de CONFIGURATION (nom absent de servo_axes) :
            # elle se repeterait a chaque periode, donc une seule trace.
            self._warnOnce("nudge_refuse", "nudge refuse : %s" % res.reason)

    def _startTraining(self):
        if self._trainGoal is not None and not self._trainGoal.done():
            self.get_logger().warn("apprentissage deja en cours : appui ignore")
            return
        if not self._trainCli.server_is_ready():
            self._warnOnce(
                "train_absent",
                "aucun serveur d'action sur %s : face_train_node ne tourne pas (il vit "
                "HORS du container de composables)." % self._trainCli._action_name)
            return
        self.get_logger().info("apprentissage : envoi du goal TrainFaces")
        self._sendMode("gen", "on")
        self._genOn = True
        self._trainGoal = self._trainCli.send_goal_async(
            TrainFaces.Goal(), feedback_callback=self._onTrainFeedback)
        self._trainGoal.add_done_callback(self._onTrainAccepted)

    def _onTrainAccepted(self, future):
        try:
            handle = future.result()
        except Exception:                                           # noqa: BLE001
            self._sendMode("gen", "off")
            self._genOn = False
            return
        if not handle.accepted:
            self._sendMode("gen", "off")
            self._genOn = False
            return
        handle.get_result_async().add_done_callback(
            lambda _f: (setattr(self, "_genOn", False), self._sendMode("gen", "off"),
                        self._publishState()))

    def _onTrainFeedback(self, fb):
        self.get_logger().info(
            "apprentissage %d/%d : %s"
            % (fb.feedback.step, fb.feedback.total, fb.feedback.line))

    # ---------------------------------------------------------------- modes
    def _loadKeymap(self):
        """Carte des touches depuis les surcharges `keymap.*` du YAML, validee.

        Les noms dependent du fichier (modes, groupes) : on les DECLARE depuis les
        surcharges, une surcharge non declaree etant invisible en Humble.
        """
        overrides = getattr(self, "_parameter_overrides", {}) or {}
        params = {}
        for name, prm in overrides.items():
            if name.split(".")[0] == PREFIX:
                params[name] = self.declare_parameter(name, prm.value).value
        try:
            km = parseKeymap(params, set(self._actions))
        except ValueError as exc:
            self._keymapError = str(exc)
            self.get_logger().error("carte des touches REFUSEE, repli conduite seule : %s" % exc)
            return dict(_FALLBACK_KEYMAP)
        if self._btn["stop"] in self._keyIdx.values():
            self._keymapError = "un index de touche de mode est celui de B (STOP)"
            self.get_logger().error("carte des touches REFUSEE : %s" % self._keymapError)
            return dict(_FALLBACK_KEYMAP)
        if not km:
            self.get_logger().warn("aucune carte `keymap.*` : repli conduite seule")
            return dict(_FALLBACK_KEYMAP)
        return km

    def _buildActions(self):
        """nom -> (execution, libelle, etat on/off ou None). Le libelle peut etre dynamique."""
        step = self._targetStep
        return {
            # TRK-P, RECO, CAPT, GEN, TRK-V n'agissent qu'ACQ ALLUMEE (2026-10-13) : elles
            # suivent ce que la detection trouve ; le HUD ne les montre pas sans elle.
            "tracking": (lambda: self._needAcq(self._actTracking, "TRK-P"),
                         lambda: "TRK-P suivi physique",
                         lambda: self._trackingOn),
            "recenter": (lambda: self._sendMode("recenter", "now"), lambda: "recentrage", None),
            "recognition": (lambda: self._needAcq(self._actRecognition, "RECO"),
                            lambda: "RECO reconnaissance",
                            lambda: self._recog in ("recognition", "acquisition")),
            "acquisition": (lambda: self._needAcq(self._actAcquisition, "CAPT"),
                            lambda: "CAPT capture",
                            lambda: self._recog == "acquisition"),
            "tracker": (lambda: self._actTracker(+1),
                        lambda: "tracker %s >" % self._trkvAlgo, None),
            "predict": (lambda: self._actPredict(+1),
                        lambda: "prediction %s >" % self._predict, None),
            "detector": (lambda: self._actDetector(+1),
                         lambda: "detecteur %s >" % self._detector, None),
            "training": (lambda: self._needAcq(self._startTraining, "GEN"),
                         lambda: "GEN apprentissage",
                         lambda: self._genOn),
            "target_grow": (lambda: self._sendMode("deadzone", "%+.3f" % step),
                            lambda: "cible +", None),
            "target_shrink": (lambda: self._sendMode("deadzone", "%+.3f" % -step),
                              lambda: "cible -", None),
            "speed_max_up": (lambda: self._bumpSpeedMax(+1),
                             lambda: "plafond %d +" % self._speedMax, None),
            "speed_max_down": (lambda: self._bumpSpeedMax(-1),
                               lambda: "plafond %d -" % self._speedMax, None),
            "speed_min_up": (lambda: self._bumpSpeedMin(+1),
                             lambda: "plancher %d +" % self._speedMin, None),
            "speed_min_down": (lambda: self._bumpSpeedMin(-1),
                               lambda: "plancher %d -" % self._speedMin, None),
            "calib_start": (lambda: self._calibStart(False), lambda: "lancer calibration",
                            self._calBusy),
            "calib_imu": (lambda: self._calibStart(True), lambda: "controle IMU", None),
            # Reglage AUTOMATIQUE des gains PID des roues : d'abord ROUES LEVEES (grille de
            # gains x plages de vitesse, roues levees VERIFIEES), puis AU SOL en ligne droite
            # avant/arriere (gains du fichier contre le meilleur jeu roues levees).
            "calib_pid_raised": (lambda: self._calibStart(False, "pid_roues_levees"),
                                 lambda: "PID roues levees", self._calBusy),
            "calib_pid_ground": (lambda: self._calibStart(False, "pid_sol"),
                                 lambda: "PID au sol (ligne droite)", self._calBusy),
            # Batterie : PLEIN = 60 s au repos, sans mouvement (chargeur debranche, apres 1 h
            # de repos) ; DECHARGE = cycles rotation/repos ROUES LEVEES jusqu'a la coupure
            # (des heures ; continue meme si la manette s'endort -- B l'arrete).
            "calib_batt_full": (lambda: self._calibStart(True, "batterie_plein"),
                                lambda: "batterie : plein au repos", self._calBusy),
            "calib_batt_discharge": (lambda: self._calibStart(False, "batterie_decharge"),
                                     lambda: "batterie : profil de decharge", self._calBusy),
            "calib_arcs": (self._calibToggleArcs, lambda: "arcs", lambda: self._calArcs),
            "calib_variant": (self._calibNextVariant,
                              lambda: "variante %s" % self._calVariant, None),
            "calib_apply": (self._calibApply, lambda: "appliquer en RAM",
                            lambda: self._calApplied),
            "calib_revert": (self._calibRevert, lambda: "revenir au fichier", None),
            "calib_pi": (self._calibTogglePi, lambda: "PI de lacet", lambda: self._calPi),
            # --- chaine de TRACKING : activation des fonctions -------------------------
            "acq": (self._actAcq, lambda: "ACQ detection", lambda: self._acqOn),
            # ACQ + TRK-V + TRK-P + RECO d'un geste (A long, 2026-10-13) : tout COUPE si l'une
            # tourne, sinon tout ALLUME.
            "chain_all": (self._actChainAll, lambda: "ACQ+TRK-V+TRK-P+RECO",
                          lambda: self._chainAny()),
            "trkv": (lambda: self._needAcq(self._actTrkv, "TRK-V"), lambda: "TRK-V suivi video",
                     lambda: self._trkvOn),
            # --- chaine de TRACKING : algorithme du sous-mode (ZL / ZR) ----------------
            "detector_prev": (lambda: self._actDetector(-1),
                              lambda: "< detecteur %s" % self._detector, None),
            "tracker_prev": (lambda: self._actTracker(-1),
                             lambda: "< tracker %s" % self._trkvAlgo, None),
            "predict_prev": (lambda: self._actPredict(-1),
                             lambda: "< prediction %s" % self._predict, None),
            "capture_next": (lambda: self._actCapture(+1),
                             lambda: "duree %s >" % self._captText(), None),
            "capture_prev": (lambda: self._actCapture(-1),
                             lambda: "< duree %s" % self._captText(), None),
            "flip_image": (self._actFlip, lambda: "miroir image",
                           lambda: bool(self._flipOn)),
            # --- mode NAV : sous-modes, carte courante, gates, destinations ----------------
            # (nav_supervisor). La carte COURANTE est gardee d'un sous-mode a l'autre ; la
            # SELECTION (L / R) est tenue par le superviseur, que le HUD et le MCP lisent aussi.
            "nav_explore": (lambda: self._nav("exploration"), lambda: "cartographier",
                            lambda: self._navSub() == "exploration"),
            # Exploration AUTOMATIQUE (gates, retour au depart) : bascule marche / arret.
            "nav_auto_explore": (lambda: self._nav("auto_explore"),
                                 lambda: "explorer seul",
                                 lambda: bool(self._navState and self._navState.detail
                                              .startswith("exploration auto :"))),
            "nav_new_map": (lambda: self._nav("new_map"), lambda: "nouvelle carte", None),
            # A long (2026-10-13) : allumer / eteindre la NAVIGATION (pile a la demande).
            "nav_toggle": (self._actNavToggle,
                           lambda: "eteindre NAV" if self._navOn() else "allumer NAV",
                           self._navOn),
            # A en navigation : RONDE des destinations, en boucle, dans l'ordre le plus court.
            "nav_patrol": (lambda: self._nav("patrol"), lambda: "ronde des destinations",
                           lambda: bool(self._navState and getattr(
                               self._navState, "auto_state", "") == "patrol")),
            "nav_dest_del_confirm": (self._askDeleteDest,
                                     lambda: "supprimer %s" % self._navSel("dest"), None),
            "nav_gate_next": (lambda: self._nav("select", "gate+"),
                              lambda: "gate %s >" % self._navSel("gate"), None),
            "nav_gate_prev": (lambda: self._nav("select", "gate-"),
                              lambda: "< gate %s" % self._navSel("gate"), None),
            "nav_explore_gate": (lambda: self._nav("explore_gate"),
                                 lambda: "aller a %s" % self._navSel("gate"),
                                 lambda: bool(self._navState and getattr(
                                     self._navState, "auto_state", "") in ("gate", "home"))),
            # Choix de la CARTE (+ / -) parmi les cartes enregistrees, appliquee 2 s apres le
            # dernier appui : carte FORCEE (relocalisation sur elle seule).
            "nav_map_next": (lambda: self._nav("select", "map+"),
                             lambda: "carte %s >" % self._navMap(), None),
            "nav_map_prev": (lambda: self._nav("select", "map-"),
                             lambda: "< carte %s" % self._navMap(), None),
            # Suppression de la carte : DEMANDE de confirmation (boite au centre du HUD).
            "nav_map_delete": (self._askDeleteMap,
                               lambda: "supprimer carte %s" % self._navMap(), None),
            "nav_save_map": (lambda: self._nav("save_map"), lambda: "sauver la carte", None),
            "nav_idle": (lambda: self._nav("idle"), lambda: "arreter", None),
            "nav_navigate": (lambda: self._nav("navigation"),
                             lambda: "naviguer (carte courante)",
                             lambda: self._navSub() == "navigation"),
            "nav_save_wp": (lambda: self._nav("dest_add"), lambda: "nouvelle destination",
                            None),
            "nav_dest_next": (lambda: self._nav("select", "dest+"),
                              lambda: "dest. %s >" % self._navSel("dest"), None),
            "nav_dest_prev": (lambda: self._nav("select", "dest-"),
                              lambda: "< dest. %s" % self._navSel("dest"), None),
            "nav_goto_sel": (lambda: self._nav("goto"),
                             lambda: "aller a %s" % self._navSel("dest"),
                             lambda: bool(self._navState and
                                          self._navState.goal_status == "active")),
            "nav_dest_del": (lambda: self._nav("dest_del"),
                             lambda: "supprimer %s" % self._navSel("dest"), None),
            "nav_relocalize": (lambda: self._nav("relocalize"), lambda: "relocaliser",
                               lambda: bool(self._navState and
                                            getattr(self._navState, "loc_state", "") ==
                                            "relocalisation")),
            "nav_goto_next": (lambda: self._nav("goto", "next"), lambda: "aller au point suivant",
                              lambda: bool(self._navState and
                                           self._navState.goal_status == "active")),
            "nav_goto_home": (lambda: self._nav("goto", "home"), lambda: "retour au depart",
                              None),
            "nav_cancel": (lambda: self._nav("cancel"), lambda: "annuler le but", None),
            "nav_unavailable": (lambda: self.get_logger().warn(
                "navigation : pas encore de lot NAV"), lambda: "(lot NAV a venir)", None),
        }

    def _run(self, name):
        # L'action en cours d'execution : _nav() la marque EN COURS jusqu'a la reponse.
        self._curAction = name
        try:
            self._actions[name][0]()
        finally:
            self._curAction = None
        self._publishState()

    def _keyValue(self, msg, key):
        if key == "ZL":
            return 1 if self._axisRaw(msg, self._axTrigL) > 0.5 else 0
        if key == "ZR":
            return 1 if self._axisRaw(msg, self._axTrigR) > 0.5 else 0
        return self._button(msg, self._keyIdx[key])

    def _groups(self):
        return (self._keymap.get(self._mode) or {}).get("groups", [])

    def _groupName(self):
        g = self._groups()
        return g[self._group % len(g)] if g else ""

    def _binding(self):
        m = self._keymap.get(self._mode) or {}
        return m.get("bind", {}).get(self._groupName(), {})

    def _cycleGroup(self, step):
        g = self._groups()
        if len(g) > 1:
            self._group = (self._group + step) % len(g)
            self.get_logger().info("groupe %s / %s" % (self._mode, self._groupName()))
            # NAV allumee : le groupe EST le sous-mode (exploration <-> navigation, carte et
            # pose transmises, sans relocalisation).
            if self._mode == "navigation" and self._navOn() and \
                    self._groupName() in ("exploration", "navigation"):
                self._nav(self._groupName())
        self._publishState()

    def _requestMode(self, which):
        if self._modeInFlight:
            return
        if not self._modeCli.service_is_ready():
            self._warnOnce("mode_manager_absent",
                           "aucun gestionnaire de modes sur %s : on reste en %s"
                           % (self._modeCli.srv_name, self._mode))
            return
        self._calibCancel("changement de mode")
        self._modeInFlight = True
        self._modeCli.call_async(ChangeMode.Request(mode=which)).add_done_callback(
            self._onModeDone)

    def _onModeDone(self, future):
        self._modeInFlight = False
        try:
            res = future.result()
        except Exception as exc:                                    # noqa: BLE001
            self.get_logger().warn("changement de mode echoue : %s" % exc)
            return
        if not res.success:
            self.get_logger().warn("mode refuse : %s" % res.message)

    def _onRobotMode(self, m):
        """Le mode APPLIQUE, tenu par le gestionnaire : le seul qui fasse foi ici."""
        if m.mode != self._mode:
            # Sortir de NAV ne l'ETEINT PAS (demande utilisateur 2026-10-13) : la pile reste
            # allumee ; seul A long, de retour en NAV, l'arrete. (Le superviseur arrete tout de
            # meme ses buts et automatismes hors du mode NAV, et sauve la carte.)
            self._group = 0
            # Le gestionnaire vient de poser detect_enabled -- FAUX dans tous les modes depuis
            # le 2026-10-13, TRACKING compris : la detection ne part qu'a la touche ACQ. On le
            # reflete, et on le dit au HUD (pastille ACQ).
            self._acqOn = False
            self._sendMode("detect", "on" if self._acqOn else "off")
        self._mode, self._modeLabel = m.mode, m.label or m.mode.upper()
        if self._mode not in self._keymap:
            self._warnOnce("keymap_" + self._mode,
                           "mode %s sans carte de touches : seules B et la croix agissent"
                           % self._mode)
        self.get_logger().info("mode du robot : %s%s" % (self._modeLabel,
                                                          " (DEGRADE)" if m.degraded else ""))
        self._publishState()

    def _publishState(self):
        st = TeleopState()
        st.header.stamp = self.get_clock().now().to_msg()
        st.mode = self._modeLabel
        st.group = self._groupName()
        st.groups = list(self._groups())
        for key, action in sorted(self._binding().items(),
                                  key=lambda kv: (KEYS.index(kv[0].replace(LONG, "")),
                                                  kv[0].endswith(LONG))):
            _fn, label, active = self._actions[action]
            st.keys.append(displayKey(key))
            st.labels.append(label())
            st.active.append(bool(active()) if active is not None else False)
            st.pending.append(action in self._pendingActs)
        st.armed = self._motionOn
        if hasattr(st, "nav_stack"):
            # Dans TOUS les modes : la pile NAV reste allumee hors du mode NAV.
            st.nav_stack = self._navStackState()
        if hasattr(st, "confirm_text"):
            st.confirm_text = self._confirm["text"] if self._confirm is not None else ""
        st.busy = self._calBusy()
        st.busy_text = self._calText if st.busy else ""
        self._statePub.publish(st)

    # ---------------------------------------------------------- actions de mode
    def _actTracking(self):
        self._trackingOn = not self._trackingOn
        self._sendMode("tracking", "on" if self._trackingOn else "off")

    def _actRecognition(self):
        # Bascule off <-> recognition. Depuis "acquisition" on retombe sur "off" : les deux
        # modes sont exclusifs cote facerecog_node, et cette action est celle qui COUPE.
        self._recog = "off" if self._recog != "off" else "recognition"
        self._sendMode("recognition", self._recog if self._recog != "off" else "off")

    def _actAcquisition(self):
        if self._recog == "acquisition":
            self._recog = "recognition"
            self._sendMode("recognition", "recognition")
        else:
            self._recog = "acquisition"
            self._sendMode("acquisition", "acquisition")

    @staticmethod
    def _step(values, current, step):
        try:
            return values[(values.index(current) + step) % len(values)]
        except ValueError:
            return values[0]

    def _actTracker(self, step=1):
        """Algorithme de TRK-V. N'est ENVOYE que si TRK-V est actif (sinon tracker=none)."""
        self._trkvAlgo = self._step(_TRK_ALGOS, self._trkvAlgo, step)
        if self._trkvOn:
            self._tracker = self._trkvAlgo
            self._sendMode("tracker", self._tracker)

    def _actTrkv(self):
        """TRK-V on/off : off = tracker `none` (verrou tenu par la detection seule)."""
        self._trkvOn = not self._trkvOn
        self._tracker = self._trkvAlgo if self._trkvOn else "none"
        self._sendMode("tracker", self._tracker)

    def _actPredict(self, step=1):
        self._predict = self._step(_PREDICTS, self._predict, step)
        self._sendMode("predict", self._predict)

    def _captText(self):
        return "illimitee" if self._captDuration == 0 else "%d s" % self._captDuration

    def _actCapture(self, step):
        """Duree de CAPT (facerecog_node, ModeCmd `capture_duration`)."""
        self._captDuration = self._step(_CAPT_DURATIONS, self._captDuration, step)
        self._sendMode("capture_duration", str(self._captDuration))

    def _chainAny(self):
        return self._acqOn or self._trkvOn or self._trackingOn or self._recog != "off"

    def _actChainAll(self):
        """Toute la chaine d'un geste : COUPE si une fonction tourne, sinon ALLUMEE."""
        on = not self._chainAny()
        self._setAcq(on)
        if self._trkvOn != on:
            self._actTrkv()
        if self._trackingOn != on:
            self._actTracking()
        if on and self._recog == "off":
            self._recog = "recognition"
            self._sendMode("recognition", "recognition")
        elif not on and self._recog != "off":
            self._recog = "off"
            self._sendMode("recognition", "off")
        self.get_logger().info("chaine de tracking %s" % ("ALLUMEE" if on else "COUPEE"))

    def _needAcq(self, fn, name):
        """Une fonction de la chaine ne part qu'ACQ allumee (demande utilisateur 2026-10-13)."""
        if not self._acqOn:
            self.get_logger().warn("%s : allumer d'abord ACQ (A)" % name)
            return
        fn()

    def _actAcq(self):
        """ACQ on/off : `detect_enabled` de tracking_node, puis ModeCmd `detect` pour le HUD.
        Couper ACQ coupe aussi TRK-V, TRK-P, RECO et CAPT, qui en dependent."""
        if self._acqOn:
            if self._trkvOn:
                self._actTrkv()
            if self._trackingOn:
                self._actTracking()
            if self._recog != "off":
                self._recog = "off"
                self._sendMode("recognition", "off")
        self._setAcq(not self._acqOn)

    def _setAcq(self, on):
        if not self._flipSet.service_is_ready():
            self.get_logger().warn("ACQ : %s absent (groupe suivi arrete ?)" % self._trackNode)
            return
        v = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(on))
        req = SetParameters.Request(parameters=[Parameter(name="detect_enabled", value=v)])

        def done(f):
            try:
                ok = f.result().results[0].successful
            except Exception:                                   # noqa: BLE001
                ok = False
            if ok:
                self._acqOn = bool(on)
                self._sendMode("detect", "on" if on else "off")
            else:
                self.get_logger().warn("ACQ : detect_enabled refuse par %s" % self._trackNode)
            self._publishState()
        self._flipSet.call_async(req).add_done_callback(done)

    def _navSub(self):
        return self._navState.submode if self._navState is not None else ""

    # ----------------------------------------------------------- confirmation
    # Une action IRREVERSIBLE (supprimer une carte) passe par une demande : boite au centre
    # du HUD (TeleopState.confirm_text), A = oui, B (ou toute autre touche) = non, 10 s sans
    # reponse = non. Demande utilisateur 2026-10-13.
    CONFIRM_TIMEOUT_S = 10.0

    def _askDeleteMap(self):
        st = self._navState
        name = st.map_name if st is not None else ""
        if not name:
            self.get_logger().warn("navigation : aucune carte courante a supprimer")
            return
        if getattr(st, "selected_kind", "") == "map":
            self.get_logger().warn("navigation : carte choisie au + / - pas encore appliquee")
            return
        self._askConfirm("Supprimer la carte %s ?" % name.replace("carte_", ""),
                         lambda n=name: self._nav("delete_map", n))

    def _askConfirm(self, text, fn):
        self._confirm = {"text": text, "fn": fn, "t": time.monotonic(), "ready": False}
        self.get_logger().info("confirmation demandee : %s (A = oui, B = non)" % text)
        self._publishState()

    def _confirmEnd(self, why):
        self.get_logger().info("confirmation %s : %s" % (why, self._confirm["text"]))
        self._confirm = None
        self._publishState()

    def _confirmJoy(self, msg, now):
        """Touches pendant une confirmation : A seule = oui, toute autre = non. Les touches
        encore TENUES a l'ouverture (le - long) doivent d'abord etre relachees ; celles qui
        ont repondu sont ignorees jusqu'a leur relachement, sinon A partirait aussi en appui
        court (exploration auto) au relachement."""
        c = self._confirm
        if now - c["t"] > self.CONFIRM_TIMEOUT_S:
            self._confirmEnd("abandonnee (sans reponse)")
            return
        vals = {k: self._keyValue(msg, k) for k in KEYS}
        pressed = [k for k, v in vals.items() if v]
        if not c["ready"]:
            c["ready"] = not pressed
            return
        if not pressed:
            return
        self._confirmIgnore = set(pressed)
        if pressed == ["A"]:
            fn = c["fn"]
            self._confirmEnd("ACCEPTEE")
            fn()
        else:
            self._confirmEnd("annulee")

    def _navMap(self):
        """Carte en cours de choix (+ / -), sinon la carte courante ; sans le prefixe."""
        st = self._navState
        if st is None:
            return "-"
        name = st.selected_name if getattr(st, "selected_kind", "") == "map" else st.map_name
        return (name or "-").replace("carte_", "")

    def _navSel(self, kind):
        """Libelle de la selection du superviseur (gate ou destination) pour le HUD : numero
        (et nom d'une destination), "-" si rien. Champs de NavState du 2026-10-13 : lus avec
        getattr tant que bamboo_interfaces n'est pas recompile partout."""
        st = self._navState
        if st is None or getattr(st, "selected_kind", "") != kind:
            return "-"
        if not st.selected_n:
            # Numero 0 : le DEPART, en tete des deux listes (2026-10-13).
            return "depart" if st.selected_name == "depart" else "-"
        if kind == "dest" and st.selected_name and st.selected_name != "p%d" % st.selected_n:
            return "%d %s" % (st.selected_n, st.selected_name)
        return ("p%d" if kind == "dest" else "%d") % st.selected_n

    def _onNavState(self, m):
        self._navState = m
        self._navStateT = time.monotonic()

    def _navOn(self):
        """Pile NAV allumee : superviseur vivant et en exploration ou en navigation."""
        st = self._navState
        return st is not None and time.monotonic() - self._navStateT < 3.0 and \
            st.submode in ("exploration", "navigation")

    def _navStackState(self):
        """off | starting | on | stopping, pour la pastille NAV du HUD."""
        if self._navStopT is not None:
            return "stopping"
        if self._navOn():
            return "on"
        return "starting" if self._navWant else "off"

    def _navStackCall(self, on):
        if not self._navStackCli.service_is_ready():
            self.get_logger().warn("navigation : %s absent (gestionnaire de modes ?)"
                                   % self._navStackCli.srv_name)
            return

        def done(f):
            try:
                r = f.result()
                self.get_logger().info("pile NAV : %s" % r.message)
            except Exception as exc:                                # noqa: BLE001
                self.get_logger().warn("pile NAV : %s" % exc)
        self._navStackCli.call_async(SetBool.Request(data=bool(on))).add_done_callback(done)

    def _actNavToggle(self):
        """A long en NAV : allumer la navigation (pile a la demande + sous-mode du GROUPE
        courant, le robot attend un ordre) ou l'eteindre (carte sauvee, robot arrete, pile
        arretee)."""
        if self._navOn() or (self._navWant and self._navStartSent):
            self._navWant, self._navStopT = False, time.monotonic()
            if self._navCli.service_is_ready():
                self._nav("idle", then=lambda _r: self._navStackCall(False))
            else:
                self._navStackCall(False)
        else:
            self._navWant, self._navStartSent, self._navStopT = True, False, None
            self._navStarted = False
            self._navWantT, self._navRetryAt = time.monotonic(), 0.0
            self._navStackCall(True)
            self._navStackTick()
        self._publishState()

    def _navStackTick(self):
        """Demarrage : des que le superviseur repond, lui demander le sous-mode du groupe.
        Arret : fini quand le superviseur s'est tu."""
        if self._navWant and not self._navStartSent and self._navCli.service_is_ready() \
                and time.monotonic() >= self._navRetryAt:
            group = self._groupName() if self._mode == "navigation" else "exploration"
            self._navStartSent, self._navSentT = True, time.monotonic()
            self._nav(group if group in ("exploration", "navigation") else "exploration",
                      then=self._onNavStarted)
        if self._navWant and self._navOn():
            self._navStarted, self._navOffSince = True, None
        elif self._navWant and self._navStarted and not self._navOn():
            # Allumee PUIS retombee en idle sans qu'on le demande (ex. carte forcee au + / -
            # ou le robot n'est pas reconnu) : la pastille NAV redevient grise, avec la raison.
            # 60 s de grace : forcer une carte passe par idle le temps de relancer slam.
            self._navOffSince = self._navOffSince or time.monotonic()
            if time.monotonic() - self._navOffSince < 60.0:
                return
            self._navOffSince = None
            st = self._navState
            self._navWant, self._navStarted = False, False
            self.get_logger().warn("navigation : retombee a l'arret (%s)"
                                   % (st.detail if st is not None else "superviseur muet"))
            self._publishState()
        elif self._navWant and self._navStartSent and not self._navOn():
            now = time.monotonic()
            if now - self._navWantT > self.NAV_START_TIMEOUT_S:
                self._onNavStarted(None)           # abandon, avec le message
            elif now - self._navSentT > self.NAV_ANSWER_TIMEOUT_S:
                # Pas de reponse (superviseur redemarre pendant la commande, 2026-10-13) :
                # on redemande.
                self._navStartSent = False
        if self._navStopT is not None and (not self._navCli.service_is_ready() or
                                           time.monotonic() - self._navStopT > 20.0):
            self._navStopT = None
            self._publishState()

    # Demarrage complet : slam + relocalisation + Nav2 (~1 min sur un RPi4 charge).
    NAV_START_TIMEOUT_S = 180.0
    NAV_ANSWER_TIMEOUT_S = 75.0

    def _onNavStarted(self, r):
        """Le superviseur refuse tant que le lidar demarre (« pas de scan lidar ») : on
        REESSAIE toutes les 3 s, jusqu'a NAV_START_TIMEOUT_S (2026-10-13 : un seul essai, la
        pastille NAV clignotait pour toujours)."""
        if r is not None and r.success or not self._navWant:
            return
        if time.monotonic() - self._navWantT > self.NAV_START_TIMEOUT_S:
            self._navWant = False
            self.get_logger().warn("navigation : demarrage ABANDONNE apres %.0f s (%s)"
                                   % (self.NAV_START_TIMEOUT_S,
                                      r.message if r is not None else "pas de reponse"))
            self._publishState()
            return
        self._navStartSent = False
        self._navRetryAt = time.monotonic() + 3.0

    def _askDeleteDest(self):
        st = self._navState
        if st is None or getattr(st, "selected_kind", "") != "dest" or not st.selected_n:
            self.get_logger().warn("navigation : aucune destination choisie (L / R)"
                                   if st is None or st.selected_name != "depart"
                                   else "navigation : le depart ne se supprime pas")
            return
        n, name = st.selected_n, st.selected_name or "p%d" % st.selected_n
        self._askConfirm("Supprimer la destination %s ?" % name,
                         lambda k=n: self._nav("dest_del", str(k)))

    def _nav(self, command, arg="", then=None):
        """Commande au superviseur de navigation ; reponse au journal (et donc au HUD).

        L'action est EN COURS (voyant clignotant au HUD) jusqu'a la reponse : le superviseur
        ne repond qu'une fois Nav2 / slam_toolbox demarres ou arretes (10 a 30 s). Demande
        utilisateur du 2026-10-11 : savoir que l'appui a ete pris et ou en est la fonction."""
        if not self._navCli.service_is_ready():
            self.get_logger().warn("navigation : %s absent (conteneur navigation arrete ?)"
                                   % self._navCli.srv_name)
            return
        act = self._curAction
        if act:
            self._pendingActs.add(act)

        def done(f):
            self._pendingActs.discard(act)
            try:
                r = f.result()
                # DEUX appels distincts, pas `(info if ok else warn)(...)` : rclpy refuse qu'un
                # meme point d'appel change de severite ("Logger severity cannot be changed
                # between calls"), et cette exception MASQUAIT la vraie reponse (essai E2c).
                if r.success:
                    self.get_logger().info("navigation %s : %s" % (command, r.message))
                else:
                    self.get_logger().warn("navigation %s : %s" % (command, r.message))
            except Exception as exc:                                # noqa: BLE001
                r = None
                self.get_logger().warn("navigation %s echouee : %s" % (command, exc))
            if then is not None:
                then(r)
            self._publishState()
        self._navCli.call_async(NavCommand.Request(command=command, arg=arg)) \
            .add_done_callback(done)

    def _actFlip(self):
        """Bascule le miroir HORIZONTAL de l'image (`flip` de tracking_node : h <-> none).

        REFUSE sur un robot a NACELLE : le triplet (flip, invert_pan, invert_tilt) est
        indivisible (cf. videotracking.yaml) -- miroiter sans inverser le pan ferait FUIR
        le visage a la camera. Sur une camera fixe (BambooWS), rien ne depend du sens.
        On relit d'abord la valeur en vigueur : un autre client (MCP) a pu la changer.
        """
        if self._hasGimbal:
            self.get_logger().warn("miroir refuse : robot a nacelle, le pan s'inverserait "
                                   "(regler flip ET invert_pan ensemble, a l'arret)")
            return
        if not (self._flipGet.service_is_ready() and self._flipSet.service_is_ready()):
            self.get_logger().warn("miroir : %s absent (groupe suivi arrete ?)" % self._trackNode)
            return

        def got(f):
            try:
                cur = f.result().values[0].string_value
            except Exception as exc:                                # noqa: BLE001
                self.get_logger().warn("miroir : lecture de flip echouee : %s" % exc)
                return
            want = "none" if cur == "h" else "h"
            v = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=want)
            req = SetParameters.Request(parameters=[Parameter(name="flip", value=v)])

            def done(g):
                try:
                    ok = g.result().results[0].successful
                except Exception:                                   # noqa: BLE001
                    ok = False
                if ok:
                    self._flipOn = (want == "h")
                self.get_logger().info("miroir image : %s" % (
                    ("ACTIF" if want == "h" else "coupe") if ok else "REFUSE"))
                self._publishState()
            self._flipSet.call_async(req).add_done_callback(done)
        self._flipGet.call_async(GetParameters.Request(names=["flip"])).add_done_callback(got)

    def _actDetector(self, step=1):
        self._detector = self._step(_DETECTORS, self._detector, step)
        self._sendMode("detector", self._detector)

    # ------------------------------------------------------------- calibration
    def _calBusy(self):
        return self._calGoal is not None

    def _calibStart(self, imu_only, procedure=""):
        if self._calBusy():
            self.get_logger().warn("calibration deja en cours : appui ignore")
            return
        if not imu_only and not self._motionOn:
            # Le robot va TOURNER : on exige l'armement a la manette, geste volontaire.
            self.get_logger().warn("calibration refusee : traction desarmee (appui long B)")
            return
        if not self._calCli.server_is_ready():
            self._warnOnce("calib_absent", "aucun serveur %s : bamboo_calibration ne tourne "
                                           "pas (calibration_enabled=false ?)"
                           % self._calCli._action_name)
            return
        self._calProc = procedure
        goal = Calibrate.Goal(procedure=procedure, variant=self._calVariant,
                              include_arcs=self._calArcs, imu_check_only=bool(imu_only))
        self._calText = "CALIB IMU" if imu_only else (
            "PID 0/?" if procedure else "CALIB 0/?")
        self.get_logger().warn("calibration %s lancee (variante %s, arcs %s)"
                               % ("IMU SEULE" if imu_only else "COMPLETE", self._calVariant,
                                  self._calArcs))
        self._calGoal = self._calCli.send_goal_async(goal,
                                                     feedback_callback=self._onCalFeedback)
        self._calGoal.add_done_callback(self._onCalAccepted)

    def _onCalAccepted(self, future):
        try:
            handle = future.result()
        except Exception as exc:                                    # noqa: BLE001
            self.get_logger().error("calibration : envoi echoue : %s" % exc)
            self._calGoal = None
            self._publishState()
            return
        if not handle.accepted:
            self.get_logger().warn("calibration REFUSEE par bamboo_calibration")
            self._calGoal = None
            self._publishState()
            return
        self._calHandle = handle
        handle.get_result_async().add_done_callback(self._onCalResult)

    def _onCalFeedback(self, fb):
        f = fb.feedback
        self._calText = "%s %d/%d" % ("PID" if self._calText.startswith("PID") else "CALIB",
                                      f.step, f.total)
        self.get_logger().info("calibration %d/%d : %s" % (f.step, f.total, f.line))
        self._publishState()

    def _onCalResult(self, future):
        try:
            res = future.result().result
            self.get_logger().warn("calibration terminee : %s%s" % (
                "SUCCES" if res.success else "ECHEC",
                (" -> " + res.candidate_path) if res.candidate_path else ""))
        except Exception as exc:                                    # noqa: BLE001
            self.get_logger().error("calibration : resultat illisible : %s" % exc)
        self._calGoal = self._calHandle = None
        self._publishState()

    def _calibCancel(self, why):
        # Une DECHARGE de batterie dure des heures roues levees : la manette qui s'endort ne
        # l'arrete pas (le gyro du noeud de calibration arrete tout si le robot touche le
        # sol). Tout autre motif (STOP, stick, changement de mode) l'arrete.
        if why == "manette perdue" and getattr(self, "_calProc", "") == "batterie_decharge":
            return
        if self._calHandle is not None:
            self.get_logger().warn("calibration ANNULEE (%s)" % why)
            self._calHandle.cancel_goal_async()

    def _calibToggleArcs(self):
        self._calArcs = not self._calArcs

    def _calibNextVariant(self):
        self._calVariant = self._next(tuple(self._calVariants), self._calVariant)

    def _callTrigger(self, cli, what, after=None):
        if not cli.service_is_ready():
            self.get_logger().warn("%s : service %s absent" % (what, cli.srv_name))
            return

        def done(f):
            try:
                r = f.result()
                self.get_logger().info("%s : %s" % (what, r.message))
                if after is not None and r.success:
                    after()
            except Exception as exc:                                # noqa: BLE001
                self.get_logger().warn("%s echoue : %s" % (what, exc))
            self._publishState()
        cli.call_async(Trigger.Request()).add_done_callback(done)

    def _calibApply(self):
        self._callTrigger(self._calApplyCli, "candidat en RAM",
                          lambda: setattr(self, "_calApplied", True))

    def _calibRevert(self):
        def after():
            self._calApplied = False
            self._calPi = False
        self._callTrigger(self._calRevertCli, "retour au fichier", after)

    def _calibTogglePi(self):
        if not self._calPiCli.service_is_ready():
            self.get_logger().warn("PI de lacet : service %s absent" % self._calPiCli.srv_name)
            return
        want = not self._calPi

        def done(f):
            try:
                r = f.result()
                if r.success:
                    self._calPi = want
                self.get_logger().info("PI de lacet : %s" % r.message)
            except Exception as exc:                                # noqa: BLE001
                self.get_logger().warn("PI de lacet echoue : %s" % exc)
            self._publishState()
        self._calPiCli.call_async(SetBool.Request(data=want)).add_done_callback(done)

    def _warnOnce(self, key, text):
        if key in self._warned:
            return
        self._warned.add(key)
        self.get_logger().warn(text)


def main(argv=None):
    rclpy.init(args=argv)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Un STOP franc SI et seulement si on etait en train de pousser : couper l outil
        # ne doit pas laisser une derniere consigne non nulle en vol. En revanche on ne
        # recentre PAS la camera -- un servo garde sa position, et la bouger a l instant
        # ou l operateur coupe l outil est exactement ce que servocam_node evite deja en
        # desarmant le suivi.
        try:
            if node._gpDriving:
                node._stopMotion()
        except Exception:                        # noeud deja a moitie detruit : on passe
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
