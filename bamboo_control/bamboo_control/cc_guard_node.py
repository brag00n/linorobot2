#!/usr/bin/env python3
"""cc_guard -- garde-fou du Command Center : le robot ne reste jamais en mouvement sans CC ni video.

POURQUOI (2026-11-08). Le Command Center (page web, portail :8000/cc/) va pouvoir commander le robot :
missions, reglages, et plus tard le pilotage au clavier. Avant la moindre commande il faut une regle
simple : TANT QUE LE ROBOT BOUGE (traction armee ou but de navigation actif), quelqu'un doit etre
devant. Sinon il s'arrete. Decisions de l'utilisateur :
  - la page envoie un BATTEMENT DE COEUR explicite toutes les 0,5 s (/cc/heartbeat, JSON) ;
  - plus de battement depuis 1,5 s, OU video non connectee depuis 3 s  ->  STOP, but annule,
    traction desarmee, avec une ligne de journal qui dit la cause ;
  - la MANETTE N'EST JAMAIS EXIGEE : un lecteur video connecte suffit a garder le robot en route
    (c'est ce qui rend l'armement verrouille « ARM B VEROU » utilisable depuis le CC) ;
  - rien d'arme et aucun but -> le garde-fou dort : la manette seule marche comme avant.

CE NOEUD NE PILOTE PAS. Il observe, publie son etat et DECLENCHE (/cc/guard/stop) ; c'est
bamboo_teleop, seul proprietaire de l'armement, qui arrete, annule et desarme. Et il PUBLIE
l'autorisation de la voie `cc` du multiplexeur (/cc/guard/enable) : vraie seulement si la page est
la, la video aussi, et la traction armee. Aucune consigne de vitesse ne passe par ici.

Le coeur (GuardCore) est en Python pur, horloge injectee : teste sans ROS.
"""
import json
import time

# Delais decides avec l'utilisateur (2026-11-08). Le battement part toutes les 0,5 s : 1,5 s = trois
# battements manques, ce qui distingue une vraie perte d'un a-coup du Wi-Fi. La video, elle, varie
# davantage (reencodage, charge du RPi) : 3 s.
HB_TIMEOUT_S = 1.5
VIDEO_TIMEOUT_S = 3.0


class GuardCore:
    """Etat du garde-fou. Toutes les methodes prennent l'instant `now` (secondes, horloge monotone).

    Un onglet est « pilote » des qu'il envoie le premier battement : les autres sont ignores pour la
    decision (ils n'arment rien) mais comptes, pour que la page puisse dire « autre onglet ».
    """

    def __init__(self, hb_timeout=HB_TIMEOUT_S, video_timeout=VIDEO_TIMEOUT_S):
        self.hb_timeout = float(hb_timeout)
        self.video_timeout = float(video_timeout)
        self.pilot = None            # id de l'onglet qui fait foi
        self.last_hb = None          # instant du dernier battement du pilote
        self.video_ok = False        # etat de la video annonce par le pilote
        self.video_bad_since = None  # instant ou la video est devenue mauvaise (None = bonne)
        self.others = {}             # id -> instant : autres onglets vus
        self.tripped = False         # un arret a deja ete declenche pour cette perte
        self.last_reason = ""
        self.hidden = False          # derniere etat annonce par la page : onglet masque ?
        self.hidden_since = None
        self.dt_max_ms = 0.0         # plus long intervalle ENTRE ENVOIS vu par la page depuis le dernier arret
        self.last_seq = None
        self.lost_msgs = 0           # battements manquants (saut de numero) depuis le dernier arret

    # ------------------------------------------------------------------ entrees
    def heartbeat(self, tab_id, video, now, hidden=False, dt_ms=None, seq=None):
        """Un battement arrive. Renvoie True s'il vient du pilote.

        Trois indices pour DIAGNOSTIQUER un silence (2026-11-09) : `hidden` (onglet masque : le navigateur ralentit ses
        minuteries a ~1 s), `dt_ms` (temps que la PAGE a reellement attendu depuis son envoi precedent : grand = page
        ralentie, petit = envoi regulier mais message perdu en route) et `seq` (un numero saute = message PERDU)."""
        tab_id = str(tab_id or "?")
        if self.pilot is not None and tab_id != self.pilot:
            # Le pilote est silencieux depuis plus longtemps que le delai : il cede sa place,
            # sinon un onglet ferme laisserait le robot sans pilote possible.
            if self.last_hb is not None and now - self.last_hb > self.hb_timeout:
                self._takeover(tab_id)
            else:
                self.others[tab_id] = now
                return False
        if self.pilot is None:
            self._takeover(tab_id)
        self.last_hb = now
        self.others.pop(tab_id, None)
        self._setVideo(bool(video), now)
        self._diag(hidden, dt_ms, seq, now)
        return True

    def _diag(self, hidden, dt_ms, seq, now):
        hidden = bool(hidden)
        if hidden and not self.hidden:
            self.hidden_since = now
        if not hidden:
            self.hidden_since = None
        self.hidden = hidden
        try:
            dt = float(dt_ms)
            self.dt_max_ms = max(self.dt_max_ms, dt)
        except (TypeError, ValueError):
            pass
        try:
            s = int(seq)
            if self.last_seq is not None and s > self.last_seq + 1:
                self.lost_msgs += s - self.last_seq - 1
            self.last_seq = s
        except (TypeError, ValueError):
            pass

    def why_silent(self, now):
        """Phrase de diagnostic ajoutee a la cause d'un arret par silence du battement."""
        bits = []
        if self.hidden:
            bits.append("onglet MASQUE depuis %.0f s (le navigateur ralentit ses minuteries)"
                        % (now - (self.hidden_since if self.hidden_since is not None else now)))
        else:
            bits.append("onglet visible")
        if self.lost_msgs:
            bits.append("%d battement(s) PERDU(S) en route (numero de sequence saute)" % self.lost_msgs)
        elif self.dt_max_ms >= 900:
            bits.append("page ralentie : jusqu'a %.0f ms entre deux envois, aucun message perdu" % self.dt_max_ms)
        else:
            bits.append("page reguliere (max %.0f ms entre envois)" % self.dt_max_ms)
        return " ; ".join(bits)

    def _forget(self):
        self.pilot = None
        self.last_hb = None
        self.video_ok = False
        self.video_bad_since = None
        self.others = {}

    def _takeover(self, tab_id):
        self.pilot = tab_id
        self.last_hb = None
        self.video_ok = False
        self.video_bad_since = None
        self.tripped = False

    def _setVideo(self, ok, now):
        if ok:
            self.video_bad_since = None
        elif self.video_bad_since is None:
            self.video_bad_since = now        # on date le DEBUT de la mauvaise video
        self.video_ok = ok

    # ------------------------------------------------------------------ etat
    def cc_state(self, now):
        """'none' (jamais vu) | 'ok' | 'lost' (battement trop vieux)."""
        if self.pilot is None or self.last_hb is None:
            return "none"
        return "ok" if now - self.last_hb <= self.hb_timeout else "lost"

    def video_state(self, now):
        """'na' (pas de CC) | 'ok' | 'bad' (pas encore le delai) | 'lost' (delai depasse)."""
        if self.cc_state(now) != "ok":
            return "na"
        if self.video_bad_since is None:
            return "ok"
        return "lost" if now - self.video_bad_since > self.video_timeout else "bad"

    def link_ok(self, now):
        """Un CC present ET une video connectee : la condition pour qu'un pilotage soit permis."""
        return self.cc_state(now) == "ok" and self.video_state(now) in ("ok", "bad") and self.video_ok

    def evaluate(self, now, moving):
        """Decide s'il faut ARRETER le robot. Renvoie la cause (str) UNE FOIS par perte, sinon None.

        `moving` : le robot bouge ou peut bouger (traction armee, ou but de navigation actif).
        Rien d'arme -> le garde-fou dort et se reinitialise (pas d'arret pour rien).
        """
        if not moving:
            self.tripped = False
            if self.cc_state(now) == "lost":
                # Un CC disparu alors que rien ne bouge est OUBLIE : armer ensuite a la manette, HUD seul,
                # doit se comporter comme avant (sinon le fantome du dernier onglet desarmerait aussitot).
                self._forget()
            return None
        cause = None
        if self.cc_state(now) == "lost":
            cause = "CC silencieux depuis %.1f s (limite %.1f s) -- %s" % (
                now - self.last_hb, self.hb_timeout, self.why_silent(now))
        elif self.cc_state(now) == "none":
            # Arme sans CC jamais vu : on NE declenche PAS. C'est le cas normal d'un robot conduit a la
            # manette, CC ferme. Le garde-fou ne concerne que les robots que le CC a pris en charge.
            self.tripped = False
            return None
        elif self.video_state(now) == "lost":
            cause = "video perdue depuis %.1f s (limite %.1f s)" % (now - self.video_bad_since, self.video_timeout)
        if cause is None:
            self.tripped = False
            return None
        if self.tripped:
            return None                      # deja arrete pour cette perte : pas de rafale d'ordres
        self.tripped = True
        self.last_reason = cause
        self.dt_max_ms, self.lost_msgs = 0.0, 0     # le prochain arret repart d'une mesure propre
        return cause

    def snapshot(self, now, moving, armed):
        """Dictionnaire publie sur /cc/guard/state (lu par le voyant de la page)."""
        cc, vid = self.cc_state(now), self.video_state(now)
        age = None if self.last_hb is None else round(now - self.last_hb, 2)
        return {"cc": cc, "video": vid, "armed": bool(armed), "moving": bool(moving),
                "tripped": self.tripped, "reason": self.last_reason if self.tripped else "",
                "age_s": age, "others": len(self.others), "pilot": self.pilot or ""}


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from bamboo_interfaces.msg import NavState, TeleopState

    class CcGuard(Node):
        def __init__(self):
            super().__init__("cc_guard")
            p = self.declare_parameter
            self.core = GuardCore(float(p("heartbeat_timeout_s", HB_TIMEOUT_S).value),
                                  float(p("video_timeout_s", VIDEO_TIMEOUT_S).value))
            self._armed = False            # lu sur /teleop/state
            self._navGoal = False          # but actif lu sur /navigation/state
            self._navT = 0.0
            self._enabled = None           # derniere valeur publiee sur /cc/guard/enable
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, "/cc/heartbeat", self._onHeartbeat, 10)
            self.create_subscription(TeleopState, "/teleop/state", self._onTeleop, latched)
            self.create_subscription(NavState, "/navigation/state", self._onNav, 10)
            self._pubState = self.create_publisher(String, "/cc/guard/state", latched)
            self._pubStop = self.create_publisher(Bool, "/cc/guard/stop", 10)
            self._pubEnable = self.create_publisher(Bool, "/cc/guard/enable", latched)
            # Liaison saine (page + video), SANS condition d'armement : c'est ce que teleop consulte pour
            # accepter ou refuser un armement demande depuis le CC.
            self._pubLink = self.create_publisher(Bool, "/cc/guard/link", latched)
            self._link = None
            self.create_timer(0.2, self._tick)
            self._lastState = 0.0
            self.get_logger().info(
                "cc_guard : battement %.1f s, video %.1f s ; arret + but annule + desarmement, "
                "manette jamais exigee" % (self.core.hb_timeout, self.core.video_timeout))

        def _onHeartbeat(self, msg):
            try:
                d = json.loads(msg.data)
            except ValueError:
                return                                 # message mal forme : ignore, jamais d'exception
            if not isinstance(d, dict):
                return
            first = self.core.pilot is None
            lost_before = self.core.cc_state(time.monotonic()) != "ok"
            mine = self.core.heartbeat(d.get("id"), bool(d.get("video")), time.monotonic(),
                                      hidden=d.get("hidden"), dt_ms=d.get("dt"), seq=d.get("seq"))
            if mine and (first or lost_before):
                self.get_logger().info("CC connecte (onglet %s)" % self.core.pilot)

        def _onTeleop(self, msg):
            self._armed = bool(msg.armed)

        def _onNav(self, msg):
            # But actif = un nom de but et un statut « active ». Un message vieux de plus de 6 s est
            # ignore (la pile NAV est a la demande : elle peut s'etre eteinte sans prevenir).
            self._navGoal = bool(msg.goal) and msg.goal_status == "active"
            self._navT = time.monotonic()

        def _moving(self, now):
            goal = self._navGoal and now - self._navT < 6.0
            return self._armed or goal

        def _tick(self):
            now = time.monotonic()
            moving = self._moving(now)
            cause = self.core.evaluate(now, moving)
            if cause:
                # ERROR et non WARN : c'est un arret de securite, il doit se voir dans le journal du CC.
                self.get_logger().error("GARDE-FOU CC : %s -> arret, but annule, traction desarmee" % cause)
                self._pubStop.publish(Bool(data=True))
            # Autorisation de la voie `cc` du multiplexeur : liaison saine ET traction armee.
            enable = bool(self._armed and self.core.link_ok(now))
            if enable != self._enabled:
                self._enabled = enable
                self._pubEnable.publish(Bool(data=enable))
            link = bool(self.core.link_ok(now))
            if link != self._link:
                self._link = link
                self._pubLink.publish(Bool(data=link))
            if now - self._lastState >= 0.5:
                self._lastState = now
                self._pubState.publish(String(data=json.dumps(
                    self.core.snapshot(now, moving, self._armed))))

    rclpy.init(args=argv)
    node = CcGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
