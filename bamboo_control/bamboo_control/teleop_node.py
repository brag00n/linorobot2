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

PERTE DE MANETTE = ARRET DE LA CAMERA. Sans /joy depuis `joy_timeout_s`, la vitesse est
remise a zero. Avec `autorepeat_rate: 25.0` cote joy_linux, un stick tenu republie en
continu : le silence signifie donc reellement "plus de manette", et pas "stick immobile".
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from bamboo_interfaces.action import TrainFaces
from bamboo_interfaces.msg import ModeCmd
from bamboo_interfaces.srv import Nudge

# Enumerations RESOLUES ICI (cf. docstring : le consommateur refuse une valeur vide).
# A garder alignees sur face_detection.cpp (detecteurs et trackers) et sur
# tracking_node.cpp (prediction).
_TRACKERS = ("none", "mil", "vit")
_DETECTORS = ("haar", "dnn", "yunet")
_PREDICTS = ("off", "anticip", "coast")


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
        # Face Nintendo numerotee a la Xbox : le B PHYSIQUE prend l'index 0, la ou on
        # attendrait A. Releve, pas deduit.
        self._btn = {
            "tracking": self.declare_parameter("button_tracking", 3).value,
            "recognition": self.declare_parameter("button_recognition", 2).value,
            "acquisition": self.declare_parameter("button_acquisition", 1).value,
            "tracker": self.declare_parameter("button_tracker", 4).value,
            "predict": self.declare_parameter("button_predict", 5).value,
            "recenter": self.declare_parameter("button_recenter", 3).value,
            "stop": self.declare_parameter("button_stop", 0).value,
            "motion": self.declare_parameter("button_motion", 0).value,
            "speed_min_up": self.declare_parameter("button_speed_min_up", 7).value,
            "speed_min_down": self.declare_parameter("button_speed_min_down", 6).value,
        }
        # Cette manette ne DECLARE PAS de clic de stick (descripteur a 10 codes, sans
        # BTN_THUMBL/BTN_THUMBR) : les deux fonctions que le proto leur confie n'ont plus
        # de bouton libre, les 14 autres etant pris. Elles passent donc en appui LONG sur
        # un bouton qui garde son role court -- armement sur B, recentrage sur X. Le
        # partage est DEDUIT de l'egalite des index, pas code en dur : sur une manette
        # qui declare ses clics de stick, il suffit de donner des index distincts et les
        # deux fronts redeviennent independants, sans toucher a ce fichier.
        self._longPress = max(0.1, float(
            self.declare_parameter("long_press_s", 0.6).value))
        self._sharedMotion = (self._btn["motion"] == self._btn["stop"])
        self._sharedRecenter = (self._btn["recenter"] == self._btn["tracking"])

        # --- axes de servo : des NOMS, jamais un numero de voie ---------------
        self._panName = self.declare_parameter("pan_axis_name", "pan").value
        self._tiltName = self.declare_parameter("tilt_axis_name", "tilt").value
        self._invPan = -1.0 if self.declare_parameter("invert_pan", False).value else 1.0
        self._invTilt = -1.0 if self.declare_parameter("invert_tilt", True).value else 1.0

        self._maxDegS = float(self.declare_parameter("nudge_max_deg_s", 45.0).value)
        self._nudgeHz = max(1.0, float(self.declare_parameter("nudge_rate_hz", 10.0).value))
        self._minDeg = float(self.declare_parameter("nudge_min_deg", 0.3).value)
        self._nudgeOn = bool(self.declare_parameter("nudge_enabled", True).value)
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
        self._speedMax = int(self.declare_parameter("speed_max_level", 3).value)
        self._cmdHz = max(1.0, float(self.declare_parameter("cmd_vel_rate_hz", 10.0).value))
        self._motionOn = bool(self.declare_parameter("motion_enabled", False).value)
        self._targetStep = float(self.declare_parameter("target_step", 0.01).value)
        self._joyTimeout = float(self.declare_parameter("joy_timeout_s", 0.5).value)

        modeTopic = self.declare_parameter("mode_topic", "/videotracking/mode_cmd").value
        nudgeSrv = self.declare_parameter("nudge_service", "/servo/nudge").value
        trainAction = self.declare_parameter(
            "train_action", "/videotracking/train_faces").value

        # --- modele local des modes (cf. docstring) ---------------------------
        self._trackingOn = False
        self._recog = "off"              # off / recognition / acquisition
        self._tracker = _TRACKERS[1]     # mil : le seul valide avant OpenCV 4.10
        self._detector = _DETECTORS[1]   # dnn : robuste de profil
        self._predict = _PREDICTS[0]
        self._seq = 0

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

        # Latche : un noeud du groupe qui demarre APRES l'appui doit recevoir l'etat
        # demande. QoS identique cote consommateurs (profondeur 1, reliable).
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._modePub = self.create_publisher(ModeCmd, modeTopic, latched)
        self._modeSub = self.create_subscription(
            ModeCmd, modeTopic, self._onModeEcho, latched)

        # Nom LOGIQUE : c'est `control.launch.py` qui le remappe (argument
        # `cmd_vel_out_topic`). Un remappage sans clef correspondante est ignore EN
        # SILENCE, d'ou l'exposition explicite de celle-ci.
        self._velPub = self.create_publisher(Twist, "cmd_vel", 10)

        self._joySub = self.create_subscription(Joy, "joy", self._onJoy, 10)
        self._nudgeCli = self.create_client(Nudge, nudgeSrv)
        self._trainCli = ActionClient(self, TrainFaces, trainAction)

        self._timer = self.create_timer(1.0 / self._nudgeHz, self._onNudgeTick)
        self._velTimer = self.create_timer(1.0 / self._cmdHz, self._onDriveTick)
        self.get_logger().info(
            "bamboo_teleop pret : modes -> %s, nudge -> %s (%.0f deg/s a fond, %.0f Hz), "
            "apprentissage -> %s, deplacement -> %s a %.0f Hz (moteurs %s au demarrage)."
            % (modeTopic, nudgeSrv, self._maxDegS, self._nudgeHz, trainAction,
               self._velPub.topic_name, self._cmdHz,
               "ARMES" if self._motionOn else "desarmes"))

    # ---------------------------------------------------------------- entrees
    @staticmethod
    def _axisRaw(msg, idx):
        """Lecture BRUTE, pour les gachettes.

        Defaut -1.0 et non 0.0 : une gachette au repos vaut -1, et l'absence d'axe doit
        donc se lire "relachee". Ne passe PAS par la loi d'axe (cf. en-tete).
        """
        return float(msg.axes[idx]) if 0 <= idx < len(msg.axes) else -1.0

    def _axis(self, msg, idx):
        """Loi d'axe du proto (RobotMain._axis), transposee telle quelle.

        Lecture TOLERANTE : une manette plus pauvre que la table ne fait pas crasher.
        """
        v = float(msg.axes[idx]) if 0 <= idx < len(msg.axes) else 0.0
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
        return "short" if court else None

    def _onJoy(self, msg):
        now = time.monotonic()
        self._lastJoy = now

        # --- vitesse des axes servo (integree par le timer) ------------------
        self._velPan = self._invPan * self._axis(msg, self._axPan) * self._maxDegS
        self._velTilt = self._invTilt * self._axis(msg, self._axTilt) * self._maxDegS

        # --- deplacement : stick gauche --------------------------------------
        self._driveFromJoy(msg)

        if self._edge("stop", self._button(msg, self._btn["stop"]), now):
            # STOP inconditionnel : il vaut meme moteurs DESARMES, parce qu un zero franc
            # est la seule chose qu on veut pouvoir emettre sans reflechir.
            self._stopMotion()

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
                self._stopMotion()               # desarmer ARRETE, il ne fige pas
            self.get_logger().warning(
                "moteurs %s a la manette (le verrou `enable_cmd_vel` du driver reste "
                "maitre)" % ("ARMES" if self._motionOn else "desarmes"))

        if self._edge("speed_min_up", self._button(msg, self._btn["speed_min_up"]), now):
            self._bumpSpeedMin(+1)
        if self._edge("speed_min_down",
                      self._button(msg, self._btn["speed_min_down"]), now):
            self._bumpSpeedMin(-1)

        # Gachettes : lecture BRUTE (repos -1), front a > 0.5.
        if self._edge("trigger_r",
                      1 if self._axisRaw(msg, self._axTrigR) > 0.5 else 0, now):
            self._bumpSpeedMax(+1)
        if self._edge("trigger_l",
                      1 if self._axisRaw(msg, self._axTrigL) > 0.5 else 0, now):
            self._bumpSpeedMax(-1)

        # --- boutons de mode : toujours une valeur ABSOLUE -------------------
        # Suivi et recentrage partagent X sur cette manette : appui court = suivi, appui
        # long = recentrage. X est donc le SEUL bouton dont l action parte au relachement.
        # "Desarmer le suivi puis recentrer" tient alors dans un seul bouton, ce qui est
        # le geste reel -- tant que le suivi verrouille un visage, onTrack reecrit la cible
        # a chaque detection et un recentrage serait aussitot efface.
        if self._sharedRecenter:
            ev = self._holdEdge("tracking", self._button(msg, self._btn["tracking"]), now)
            if ev == "long":
                self._sendMode("recenter", "now")
            elif ev == "short":
                self._trackingOn = not self._trackingOn
                self._sendMode("tracking", "on" if self._trackingOn else "off")
        elif self._edge("tracking", self._button(msg, self._btn["tracking"]), now):
            self._trackingOn = not self._trackingOn
            self._sendMode("tracking", "on" if self._trackingOn else "off")

        if self._edge("recognition", self._button(msg, self._btn["recognition"]), now):
            # Bascule off <-> recognition. Depuis "acquisition" on retombe sur "off" :
            # les deux modes sont exclusifs cote facerecog_node, et ce bouton est celui
            # qui COUPE -- c'est la seule facon de sortir de l'acquisition en un appui.
            self._recog = "off" if self._recog != "off" else "recognition"
            self._sendMode("recognition", self._recog if self._recog != "off" else "off")

        if self._edge("acquisition", self._button(msg, self._btn["acquisition"]), now):
            if self._recog == "acquisition":
                self._recog = "recognition"
                self._sendMode("recognition", "recognition")
            else:
                self._recog = "acquisition"
                self._sendMode("acquisition", "acquisition")

        if self._edge("tracker", self._button(msg, self._btn["tracker"]), now):
            self._tracker = self._next(_TRACKERS, self._tracker)
            self._sendMode("tracker", self._tracker)

        if self._edge("predict", self._button(msg, self._btn["predict"]), now):
            self._predict = self._next(_PREDICTS, self._predict)
            self._sendMode("predict", self._predict)

        # Recentrage a bouton PROPRE : seulement si on ne le partage pas avec le suivi
        # (sinon il est deja traite en appui long, plus haut). Consomme par servocam_node
        # SEUL : sans le groupe tracking il n'y a pas de notion de position de repos cote
        # hote, donc rien ne bouge. C'est coherent, pas un bug.
        if not self._sharedRecenter and self._edge(
                "recenter", self._button(msg, self._btn["recenter"]), now):
            self._sendMode("recenter", "now")

        # --- croix directionnelle, vue comme deux boutons virtuels -----------
        # SIGNE RELEVE, et il est CONTRE-INTUITIF : sur l'axe 7 le HAUT vaut -1 et le BAS
        # +1 (convention du noyau, comme pour l'axe vertical d'un stick). Ce fichier avait
        # l'inverse avant la mesure -- le detecteur et l'apprentissage etaient echanges.
        dy = self._axis(msg, self._axDpadY)
        if self._edge("dpad_up", 1 if dy < -0.5 else 0, now):
            self._detector = self._next(_DETECTORS, self._detector)
            self._sendMode("detector", self._detector)
        if self._edge("dpad_down", 1 if dy > 0.5 else 0, now):
            self._startTraining()

        # Taille de la surface cible (proto : _resize_target, pas de 0,01). La valeur porte
        # un DELTA SIGNE : l etat vit cote serveur, ce noeud ne le duplique pas. Une cible
        # ModeCmd inconnue etant ignoree EN SILENCE, ce D-pad est sans effet tant que
        # servocam_node et overlay_node ne la consomment pas.
        dx = self._axis(msg, self._axDpadX)
        if self._edge("dpad_right", 1 if dx > 0.5 else 0, now):
            self._sendMode("deadzone", "%+.3f" % self._targetStep)
        if self._edge("dpad_left", 1 if dx < -0.5 else 0, now):
            self._sendMode("deadzone", "%+.3f" % -self._targetStep)

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
        self._lin = self._scaleAxis(fwd, fmin, fmax) * self._teleLin
        self._ang = self._scaleAxis(turn, fmin, fmax) * self._teleAng

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
            fwd = -self._axis(msg, self._axDriveY)   # avant = stick vers le HAUT (LY < 0)
            turn = -self._axis(msg, self._axDriveX)  # gauche = + (angular.z anti-horaire)
        driving = (fwd != 0.0 or turn != 0.0)
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
        if time.monotonic() - self._lastJoy > self._joyTimeout:
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
            return
        if not res.success:
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
        self._trainGoal = self._trainCli.send_goal_async(
            TrainFaces.Goal(), feedback_callback=self._onTrainFeedback)

    def _onTrainFeedback(self, fb):
        self.get_logger().info(
            "apprentissage %d/%d : %s"
            % (fb.feedback.step, fb.feedback.total, fb.feedback.line))

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
