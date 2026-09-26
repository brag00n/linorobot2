#!/usr/bin/env python3
"""bamboo_teleop -- /joy -> modes du groupe videotracking et pas relatifs des axes servo.

PERIMETRE, ET C'EST UN CHOIX EXPLICITE : ce noeud ne publie AUCUN /cmd_vel et ne
commande AUCUN moteur de traction. La traction a la manette est le lot 5 du chantier 1
(`teleop_twist_joy`, robot BambooWS, en pause) ; ici les moteurs sont verrouilles
(`enable_cmd_vel: false`, pack 12,6 V brut sur des moteurs 7,4 V nominaux). Ajouter ce
chemin maintenant serait du code d'actuation non testable.

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
        # Aucun index en dur : la table depend du mode d'appairage (XInput/DirectInput)
        # et du pilote. Defauts = convention joy_linux/xpad, << A RELEVER a T9 >>.
        self._axPan = self.declare_parameter("axis_pan", 3).value
        self._axTilt = self.declare_parameter("axis_tilt", 4).value
        self._axDpadX = self.declare_parameter("axis_dpad_x", 6).value
        self._axDpadY = self.declare_parameter("axis_dpad_y", 7).value
        self._btn = {
            "tracking": self.declare_parameter("button_tracking", 2).value,
            "recognition": self.declare_parameter("button_recognition", 3).value,
            "acquisition": self.declare_parameter("button_acquisition", 0).value,
            "tracker": self.declare_parameter("button_tracker", 4).value,
            "predict": self.declare_parameter("button_predict", 5).value,
            "recenter": self.declare_parameter("button_recenter", 10).value,
        }

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
        self._velPan = 0.0
        self._velTilt = 0.0
        self._pendPan = 0.0
        self._pendTilt = 0.0
        self._lastJoy = 0.0
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

        self._joySub = self.create_subscription(Joy, "joy", self._onJoy, 10)
        self._nudgeCli = self.create_client(Nudge, nudgeSrv)
        self._trainCli = ActionClient(self, TrainFaces, trainAction)

        self._timer = self.create_timer(1.0 / self._nudgeHz, self._onNudgeTick)
        self.get_logger().info(
            "bamboo_teleop pret : modes -> %s, nudge -> %s (%.0f deg/s a fond, %.0f Hz), "
            "apprentissage -> %s. AUCUN /cmd_vel."
            % (modeTopic, nudgeSrv, self._maxDegS, self._nudgeHz, trainAction))

    # ---------------------------------------------------------------- entrees
    @staticmethod
    def _axis(msg, idx):
        """Lecture TOLERANTE : une manette plus pauvre que la table ne fait pas crasher."""
        return float(msg.axes[idx]) if 0 <= idx < len(msg.axes) else 0.0

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

    def _onJoy(self, msg):
        now = time.monotonic()
        self._lastJoy = now

        # --- vitesse des axes servo (integree par le timer) ------------------
        self._velPan = self._invPan * self._axis(msg, self._axPan) * self._maxDegS
        self._velTilt = self._invTilt * self._axis(msg, self._axTilt) * self._maxDegS

        # --- boutons de mode : toujours une valeur ABSOLUE -------------------
        if self._edge("tracking", self._button(msg, self._btn["tracking"]), now):
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

        if self._edge("recenter", self._button(msg, self._btn["recenter"]), now):
            # Consomme par servocam_node SEUL : sans le groupe tracking il n'y a pas de
            # notion de position de repos cote hote, donc rien ne bouge. C'est coherent,
            # pas un bug.
            self._sendMode("recenter", "now")

        # --- croix directionnelle, vue comme deux boutons virtuels -----------
        dy = self._axis(msg, self._axDpadY)
        if self._edge("dpad_up", 1 if dy > 0.5 else 0, now):
            self._detector = self._next(_DETECTORS, self._detector)
            self._sendMode("detector", self._detector)
        if self._edge("dpad_down", 1 if dy < -0.5 else 0, now):
            self._startTraining()

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
                "aucun serveur sur %s : ni servocam_node ni le driver ne tourne -- les "
                "pas de stick sont accumules, pas perdus." % self._nudgeCli.srv_name)
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
        # Aucun "frein" a envoyer : ce noeud ne commande pas de moteur et un servo garde
        # sa position. Recentrer en sortant ferait bouger la camera a l'instant ou
        # l'operateur coupe l'outil -- exactement ce que servocam_node evite deja en
        # desarmant le suivi.
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
