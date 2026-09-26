#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""ros2.py - Serveur MCP (stdio) d'ANALYSE du graphe ROS2, strictement LECTURE SEULE.

Troisieme serveur MCP du projet, pendant ROS2 de `analysis.py` : la ou `analysis`
lit les fichiers de telemetrie de l'app Windows, `ros2-analysis` interroge le graphe
ROS2 qui tourne DANS LE CONTENEUR sur le RPi, via le service **rosbridge** deja present
dans le compose linorobot2 (rosbridge_suite, WebSocket JSON `:9090`). Aucun code ROS a
ecrire cote RPi : on active le service et on s'y connecte en client WebSocket.

  ROS2_BRIDGE_URL : URL du rosbridge (defaut ws://127.0.0.1:9090). Deux options reseau :
    - tunnel SSH : `ssh -L 9090:localhost:9090 <user>@<rpi>` -> laisser 127.0.0.1.
    - direct     : `ws://<IP_RPi>:9090` (conteneur en network_mode: host).

Le client Foxglove Studio de l'utilisateur se connecte EN PARALLELE sur foxglove_bridge
(`:8765`) ; les deux bridges cohabitent (chacun s'abonne aux memes topics).

Outils (tous LECTURE SEULE, mappes sur le protocole rosbridge) :
  topics / nodes / topic_type  -> services rosapi (op call_service)
  echo / hz                    -> op subscribe (fenetre courte) puis unsubscribe
  tf                           -> subscribe /tf + /tf_static (arbre des reperes)
  odom / scan / imu / battery  -> echo + mise en forme lisible
  ros2_status                  -> synthese : nb noeuds + fraicheur des topics cles

CAVEAT QoS : rosbridge souscrit par defaut en *reliable*. `/scan` et `/imu` sont publies
en *best-effort* (sensor_data) -> un abonne reliable est INCOMPATIBLE et ne recoit rien.
Les outils scan/imu le signalent si 0 message ; parade prevue (plan, risque #4) : preciser
la QoS dans l'op subscribe (support Humble) ou un mini-republisher reliable.

L'ACTION (publish /cmd_vel, call_service save_map, parametres) passe par le MEME rosbridge :
`ros2_action.py` (serveur `ros2-action`) REUTILISE la classe `Bridge` de ce fichier, aucune
infra nouvelle. C'est bien le partage du CLIENT, pas de la lecture seule : ce qui rend
`ros2-analysis` inoffensif, c'est sa table d'outils, qui n'expose aucune ecriture.

Lancement (via .mcp.json) :
    python -m ros_control.mcp.ros2
Test manuel (necessite un rosbridge joignable) :
    python -m ros_control.mcp.ros2 selftest

DEMENAGE le 2026-09-26 (lot 3.0) depuis `linorobot2_hardware/tools/robot_control/mcp/` :
ce serveur ne parle qu'au graphe ROS, il appartient donc au depot ROS. Ne restent dans le
depot firmware que les serveurs qui lisent la telemetrie de l'app Windows et ouvrent le COM.
"""
import json
import math
import os
import sys
import time

from . import _rpc

SERVER_NAME = "ros2-analysis"
_LOG = _rpc.make_log("ros2-analysis")

BRIDGE_URL = os.environ.get("ROS2_BRIDGE_URL", "ws://127.0.0.1:9090")
# Delais par defaut (s) : connexion/reponse de service, et fenetre d'ecoute d'un topic.
CALL_TIMEOUT = float(os.environ.get("ROS2_CALL_TIMEOUT", "5.0"))
ECHO_TIMEOUT = float(os.environ.get("ROS2_ECHO_TIMEOUT", "4.0"))


# ---------------------------------------------------------------------------
# Client rosbridge (WebSocket JSON) - une connexion neuve par appel MCP (sans etat)
# ---------------------------------------------------------------------------
class BridgeError(RuntimeError):
    """Erreur de dialogue rosbridge (connexion, timeout, service en echec)."""


class Bridge:
    """Client minimal du protocole rosbridge_suite. Chaque methode ouvre puis ferme
    sa connexion : le modele d'appel MCP est ponctuel (pas de session longue), et
    l'analyse en lecture seule tolere le cout d'un handshake WebSocket par appel."""

    _seq = 0

    def __init__(self, url=None, timeout=CALL_TIMEOUT):
        self.url = url or BRIDGE_URL
        self.timeout = timeout

    @classmethod
    def _next_id(cls):
        cls._seq += 1
        return "mcp-%d" % cls._seq

    def _connect(self):
        try:
            import websocket  # paquet 'websocket-client'
        except ImportError:
            raise BridgeError(
                "dependance manquante : 'websocket-client'. Installer dans tools/.venv :\n"
                "  tools/.venv/Scripts/python.exe -m pip install websocket-client")
        try:
            return websocket.create_connection(self.url, timeout=self.timeout)
        except Exception as e:
            raise BridgeError(
                "connexion a %s impossible (%s). Le conteneur RPi tourne-t-il "
                "(docker compose up rosbridge) et le port 9090 est-il joignable "
                "(tunnel SSH ou IP directe) ? Regler ROS2_BRIDGE_URL." % (self.url, e))

    def call_service(self, service, args=None):
        """Op rosbridge call_service : renvoie le dict `values` de la reponse."""
        ws = self._connect()
        try:
            cid = self._next_id()
            req = {"op": "call_service", "service": service, "id": cid}
            if args is not None:
                req["args"] = args
            ws.send(json.dumps(req))
            deadline = time.time() + self.timeout
            while time.time() < deadline:
                ws.settimeout(max(0.05, deadline - time.time()))
                try:
                    msg = json.loads(ws.recv())
                except Exception:
                    break
                if msg.get("op") == "service_response" and msg.get("id") == cid:
                    if msg.get("result") is False:
                        raise BridgeError("service %s en echec : %s"
                                          % (service, msg.get("values")))
                    return msg.get("values", {}) or {}
            raise BridgeError("pas de reponse du service %s en %ss" % (service, self.timeout))
        finally:
            try:
                ws.close()
            except Exception:
                pass

    def subscribe_collect(self, topic, window, count=None, msg_type=None):
        """Op subscribe : collecte les messages `publish` du topic pendant `window`
        secondes (ou jusqu'a `count` messages), puis unsubscribe. Renvoie la liste
        des champs `msg`. `count=1` = un echo ; `count=None` = compte sur la fenetre."""
        ws = self._connect()
        out = []
        try:
            cid = self._next_id()
            sub = {"op": "subscribe", "topic": topic, "id": cid, "queue_length": 1}
            if msg_type:
                sub["type"] = msg_type
            ws.send(json.dumps(sub))
            deadline = time.time() + window
            while time.time() < deadline:
                ws.settimeout(max(0.05, deadline - time.time()))
                try:
                    msg = json.loads(ws.recv())
                except Exception:
                    break
                if msg.get("op") == "publish" and msg.get("topic") == topic:
                    out.append(msg.get("msg", {}))
                    if count is not None and len(out) >= count:
                        break
            try:
                ws.send(json.dumps({"op": "unsubscribe", "topic": topic, "id": cid}))
            except Exception:
                pass
            return out
        finally:
            try:
                ws.close()
            except Exception:
                pass


    # --- ops d'ECRITURE (utilisees par ros2_action.py, par AUCUN outil d'ici) ------
    def publish_repeat(self, topic, msg_type, msg, hz, seconds, stop_msg=None):
        """advertise -> publie `msg` a `hz` pendant `seconds` -> publie `stop_msg` -> unadvertise.

        UNE SEULE connexion pour toute la rafale, contrairement au reste de cette classe :
        un advertise ne survit pas a la fermeture du WebSocket (rosbridge detruit le
        publisher avec le client), donc republier en ouvrant une connexion par trame
        recreerait un publisher a chaque fois -- et les abonnes verraient un pair
        apparaitre/disparaitre 10 fois par seconde.

        `stop_msg` est envoye dans un `finally` : une exception, un timeout ou une
        deconnexion en cours de rafale laisse la consigne d'ARRET partie, jamais la
        derniere consigne de mouvement. Renvoie le nombre de trames publiees.
        """
        ws = self._connect()
        sent = 0
        try:
            ws.send(json.dumps({"op": "advertise", "topic": topic, "type": msg_type,
                                "id": self._next_id()}))
            period = 1.0 / max(0.1, float(hz))
            deadline = time.time() + float(seconds)
            while time.time() < deadline:
                ws.send(json.dumps({"op": "publish", "topic": topic, "msg": msg}))
                sent += 1
                time.sleep(period)
            return sent
        finally:
            try:
                if stop_msg is not None:
                    # Trois fois : le protocole n'acquitte pas un publish, et c'est la
                    # seule trame dont la perte serait dangereuse.
                    for _ in range(3):
                        ws.send(json.dumps({"op": "publish", "topic": topic,
                                            "msg": stop_msg}))
                        time.sleep(0.05)
                ws.send(json.dumps({"op": "unadvertise", "topic": topic}))
            except Exception:
                pass
            try:
                ws.close()
            except Exception:
                pass


def _bridge():
    return Bridge()


# ---------------------------------------------------------------------------
# Helpers de mise en forme
# ---------------------------------------------------------------------------
def _quat_to_rpy(q):
    """Quaternion (dict x,y,z,w) -> (roll, pitch, yaw) en degres."""
    x, y, z, w = q.get("x", 0.0), q.get("y", 0.0), q.get("z", 0.0), q.get("w", 1.0)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sp)))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _fmt(v, nd=3):
    try:
        return round(float(v), nd)
    except (TypeError, ValueError):
        return v


# ---------------------------------------------------------------------------
# Outils MCP (tous en LECTURE SEULE)
# ---------------------------------------------------------------------------
def t_topics(args):
    v = _bridge().call_service("/rosapi/topics")
    topics = v.get("topics", [])
    types = v.get("types", [])
    if not topics:
        return "Aucun topic (le graphe ROS2 est-il demarre ?)."
    tmap = dict(zip(topics, types)) if len(types) == len(topics) else {}
    filt = args.get("filter")
    lines = ["%d topics :" % len(topics)]
    for name in sorted(topics):
        if filt and filt not in name:
            continue
        lines.append("  %-32s %s" % (name, tmap.get(name, "")))
    return "\n".join(lines)


def t_nodes(args):
    v = _bridge().call_service("/rosapi/nodes")
    nodes = v.get("nodes", [])
    if not nodes:
        return "Aucun noeud (le graphe ROS2 est-il demarre ?)."
    return "%d noeuds :\n%s" % (len(nodes), "\n".join("  " + n for n in sorted(nodes)))


def t_topic_type(args):
    topic = args.get("topic")
    if not topic:
        return "Parametre 'topic' requis."
    v = _bridge().call_service("/rosapi/topic_type", {"topic": topic})
    return "%s : %s" % (topic, v.get("type") or "(inconnu / topic absent)")


def t_node_info(args):
    """Detail d'un noeud : ce qu'il publie, ce a quoi il souscrit, ce qu'il sert.

    C'est l'outil de DIAGNOSTIC qui manquait : `topics` dit qu'un topic existe, jamais QUI
    est au bout. Un topic present sans souscripteur (ou l'inverse) est le symptome le plus
    frequent ici -- incompatibilite de QoS, ou noeud mort qui a laisse son annonce.
    """
    node = args.get("node")
    if not node:
        return "Parametre 'node' requis (nom complet, ex. /esp32_driver ; cf. outil nodes)."
    v = _bridge().call_service("/rosapi/node_details", {"node": node})
    pub = sorted(v.get("publishing", []) or [])
    sub_ = sorted(v.get("subscribing", []) or [])
    srv = sorted(v.get("services", []) or [])
    if not (pub or sub_ or srv):
        return ("%s : aucun detail. Noeud absent du graphe (orthographe ? `nodes` pour la "
                "liste exacte), ou noeud sans aucune interface." % node)
    lines = ["%s :" % node, "  publie (%d) :" % len(pub)]
    lines += ["    " + t for t in pub] or ["    (aucun)"]
    lines.append("  souscrit (%d) :" % len(sub_))
    lines += ["    " + t for t in sub_] or ["    (aucun)"]
    lines.append("  sert (%d) :" % len(srv))
    lines += ["    " + t for t in srv] or ["    (aucun)"]
    return "\n".join(lines)


def t_echo(args):
    topic = args.get("topic")
    if not topic:
        return "Parametre 'topic' requis."
    window = float(args.get("timeout", ECHO_TIMEOUT))
    msgs = _bridge().subscribe_collect(topic, window, count=1,
                                       msg_type=args.get("type"))
    if not msgs:
        return ("Aucun message sur %s en %ss. Topic inactif, ou QoS best-effort "
                "(rosbridge souscrit en reliable) -> voir caveat scan/imu." % (topic, window))
    return "%s :\n%s" % (topic, json.dumps(msgs[0], indent=2, ensure_ascii=False))


def t_hz(args):
    topic = args.get("topic")
    if not topic:
        return "Parametre 'topic' requis."
    window = float(args.get("window_s", 5.0))
    msgs = _bridge().subscribe_collect(topic, window, count=None,
                                       msg_type=args.get("type"))
    n = len(msgs)
    if n < 2:
        return ("%s : %d message(s) en %ss -> cadence non mesurable (topic inactif "
                "ou QoS incompatible)." % (topic, n, window))
    return "%s : %d messages en %ss -> ~%.1f Hz" % (topic, n, window, (n - 1) / window)


def t_tf(args):
    br = _bridge()
    edges = {}
    for topic in ("/tf", "/tf_static"):
        for m in br.subscribe_collect(topic, float(args.get("window_s", 3.0))):
            for tr in m.get("transforms", []):
                parent = (tr.get("header", {}) or {}).get("frame_id", "?")
                child = tr.get("child_frame_id", "?")
                edges[(parent, child)] = tr.get("transform", {})
    if not edges:
        return ("Aucune transforme sur /tf ni /tf_static. Un state_publisher / EKF "
                "publie-t-il le TF ? (attendu a partir du L8/L9).")
    src, dst = args.get("from"), args.get("to")
    lines = ["Arbre TF (%d aretes parent -> enfant) :" % len(edges)]
    for (p, c) in sorted(edges):
        mark = ""
        if src and dst and p == src and c == dst:
            t = edges[(p, c)].get("translation", {})
            mark = "  <= [%s,%s,%s]" % (_fmt(t.get("x")), _fmt(t.get("y")), _fmt(t.get("z")))
        lines.append("  %s -> %s%s" % (p, c, mark))
    return "\n".join(lines)


def t_odom(args):
    topic = args.get("topic", "/odom")
    msgs = _bridge().subscribe_collect(topic, ECHO_TIMEOUT, count=1)
    if not msgs:
        return "Aucun message sur %s (driver actif ? L5+)." % topic
    m = msgs[0]
    pose = ((m.get("pose") or {}).get("pose") or {})
    pos = pose.get("position", {})
    _, _, yaw = _quat_to_rpy(pose.get("orientation", {}))
    tw = ((m.get("twist") or {}).get("twist") or {})
    lin = tw.get("linear", {})
    ang = tw.get("angular", {})
    return ("%s :\n  pose  x=%s y=%s yaw=%s deg\n  twist v=%s m/s  w=%s rad/s\n  frame %s -> %s"
            % (topic, _fmt(pos.get("x")), _fmt(pos.get("y")), _fmt(yaw),
               _fmt(lin.get("x")), _fmt(ang.get("z")),
               (m.get("header", {}) or {}).get("frame_id", "?"),
               m.get("child_frame_id", "?")))


def t_scan(args):
    topic = args.get("topic", "/scan")
    msgs = _bridge().subscribe_collect(topic, ECHO_TIMEOUT, count=1)
    if not msgs:
        return ("Aucun message sur %s. LIDAR demarre (L2) ? ATTENTION QoS : /scan est "
                "best-effort ; rosbridge souscrit en reliable -> 0 message meme si le "
                "LIDAR tourne (plan risque #4 : preciser la QoS / mini-republisher)." % topic)
    m = msgs[0]
    ranges = [r for r in (m.get("ranges") or []) if isinstance(r, (int, float))]
    finite = [r for r in ranges if math.isfinite(r) and r > 0]
    a0 = m.get("angle_min", 0.0)
    a1 = m.get("angle_max", 0.0)
    return ("%s :\n  points   : %d (%d valides)\n  portee   : %s .. %s m (min/max mesures)\n"
            "  balayage : %s .. %s deg\n  rmin/rmax capteur : %s / %s m"
            % (topic, len(ranges), len(finite),
               _fmt(min(finite)) if finite else "-", _fmt(max(finite)) if finite else "-",
               _fmt(math.degrees(a0), 1), _fmt(math.degrees(a1), 1),
               _fmt(m.get("range_min")), _fmt(m.get("range_max"))))


def t_imu(args):
    topic = args.get("topic", "/imu/data")
    msgs = _bridge().subscribe_collect(topic, ECHO_TIMEOUT, count=1)
    if not msgs:
        return ("Aucun message sur %s. Driver actif (L5) ? ATTENTION QoS : /imu est "
                "best-effort (sensor_data) ; rosbridge souscrit en reliable -> 0 message "
                "(plan risque #4)." % topic)
    m = msgs[0]
    roll, pitch, yaw = _quat_to_rpy(m.get("orientation", {}))
    g = m.get("angular_velocity", {})
    a = m.get("linear_acceleration", {})
    return ("%s :\n  attitude rpy = %s / %s / %s deg\n  gyro   = %s %s %s rad/s\n"
            "  accel  = %s %s %s m/s2"
            % (topic, _fmt(roll, 1), _fmt(pitch, 1), _fmt(yaw, 1),
               _fmt(g.get("x")), _fmt(g.get("y")), _fmt(g.get("z")),
               _fmt(a.get("x")), _fmt(a.get("y")), _fmt(a.get("z"))))


def t_battery(args):
    topic = args.get("topic", "/battery")
    msgs = _bridge().subscribe_collect(topic, ECHO_TIMEOUT, count=1)
    if not msgs:
        return "Aucun message sur %s (driver actif ? L5)." % topic
    m = msgs[0]
    return ("%s :\n  tension    : %s V\n  courant    : %s A\n  pourcentage: %s"
            % (topic, _fmt(m.get("voltage")), _fmt(m.get("current")),
               _fmt(m.get("percentage"))))


# Topics cles surveilles par ros2_status (nom -> role).
_KEY_TOPICS = [
    ("/scan", "LIDAR"), ("/odom", "odometrie filtree"),
    ("/odom/unfiltered", "odom brute driver"), ("/imu/data", "IMU"),
    ("/battery", "batterie"), ("/cmd_vel", "consigne vitesse"),
    ("/tf", "reperes"), ("/image_raw/compressed", "camera"),
]


def t_ros2_status(args):
    br = _bridge()
    try:
        nodes = br.call_service("/rosapi/nodes").get("nodes", [])
        tv = br.call_service("/rosapi/topics")
    except BridgeError as e:
        return str(e)
    topics = set(tv.get("topics", []))
    lines = ["graphe    : %d noeuds, %d topics (bridge %s)"
             % (len(nodes), len(topics), br.url),
             "topics cles :"]
    for name, role in _KEY_TOPICS:
        present = "present" if name in topics else "absent"
        lines.append("  %-24s %-8s (%s)" % (name, present, role))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Declaration MCP
# ---------------------------------------------------------------------------
_TOPIC = {"type": "string", "description": "nom du topic ROS2 (ex. /scan)"}
_TYPE = {"type": "string", "description": "type de message (optionnel ; aide rosbridge "
                                          "si le topic n'est pas encore annonce)"}

TOOLS = [
    {"name": "topics",
     "description": "Liste les topics du graphe ROS2 (via rosbridge rosapi/topics) avec "
                    "leur type. Filtre optionnel (sous-chaine).",
     "inputSchema": {"type": "object", "properties": {
         "filter": {"type": "string", "description": "ne garder que les topics contenant cette chaine"}}}},
    {"name": "nodes",
     "description": "Liste les noeuds vivants du graphe ROS2 (rosapi/nodes).",
     "inputSchema": {"type": "object", "properties": {}}},
    {"name": "topic_type",
     "description": "Type de message d'un topic (rosapi/topic_type).",
     "inputSchema": {"type": "object", "properties": {"topic": _TOPIC}, "required": ["topic"]}},
    {"name": "echo",
     "description": "Prend UN message d'un topic (subscribe fenetre courte puis "
                    "unsubscribe) et l'affiche en JSON. CAVEAT : topics best-effort "
                    "(/scan, /imu) peuvent ne rien renvoyer (rosbridge = reliable).",
     "inputSchema": {"type": "object", "properties": {
         "topic": _TOPIC, "type": _TYPE,
         "timeout": {"type": "number", "description": "fenetre d'ecoute en s (defaut 4)"}},
         "required": ["topic"]}},
    {"name": "hz",
     "description": "Cadence approximative d'un topic : compte les messages sur une "
                    "fenetre et en deduit des Hz.",
     "inputSchema": {"type": "object", "properties": {
         "topic": _TOPIC, "type": _TYPE,
         "window_s": {"type": "number", "description": "fenetre de comptage en s (defaut 5)"}},
         "required": ["topic"]}},
    {"name": "tf",
     "description": "Arbre TF : ecoute /tf + /tf_static et liste les aretes parent -> "
                    "enfant. Avec from/to, affiche la translation de cette arete si vue.",
     "inputSchema": {"type": "object", "properties": {
         "from": {"type": "string", "description": "repere parent a mettre en avant"},
         "to": {"type": "string", "description": "repere enfant a mettre en avant"},
         "window_s": {"type": "number", "description": "fenetre d'ecoute en s (defaut 3)"}}}},
    {"name": "odom",
     "description": "Resume d'un topic Odometry (defaut /odom) : pose x/y/yaw et twist v/w.",
     "inputSchema": {"type": "object", "properties": {
         "topic": {"type": "string", "description": "topic odom (defaut /odom)"}}}},
    {"name": "scan",
     "description": "Resume d'un LaserScan (defaut /scan) : nb points, points valides, "
                    "portee min/max, plage angulaire. CAVEAT QoS best-effort.",
     "inputSchema": {"type": "object", "properties": {
         "topic": {"type": "string", "description": "topic scan (defaut /scan)"}}}},
    {"name": "imu",
     "description": "Resume d'un Imu (defaut /imu/data) : attitude rpy, gyro, accel. "
                    "CAVEAT QoS best-effort.",
     "inputSchema": {"type": "object", "properties": {
         "topic": {"type": "string", "description": "topic imu (defaut /imu/data)"}}}},
    {"name": "battery",
     "description": "Resume d'un BatteryState (defaut /battery) : tension, courant, pourcentage.",
     "inputSchema": {"type": "object", "properties": {
         "topic": {"type": "string", "description": "topic batterie (defaut /battery)"}}}},
    {"name": "node_info",
     "description": "Detail d'un noeud (rosapi/node_details) : topics publies, topics "
                    "souscrits, services servis. Repond a \"qui est au bout de ce "
                    "topic ?\", ce que l'outil topics ne dit pas.",
     "inputSchema": {"type": "object", "properties": {
         "node": {"type": "string",
                  "description": "nom complet du noeud (ex. /esp32_driver ; cf. outil nodes)"}},
         "required": ["node"]}},
    {"name": "ros2_status",
     "description": "Synthese du graphe : nombre de noeuds/topics et presence des topics "
                    "cles du robot (scan, odom, imu, battery, cmd_vel, tf, camera).",
     "inputSchema": {"type": "object", "properties": {}}},
]

HANDLERS = {
    "topics": t_topics,
    "nodes": t_nodes,
    "topic_type": t_topic_type,
    "echo": t_echo,
    "hz": t_hz,
    "tf": t_tf,
    "odom": t_odom,
    "scan": t_scan,
    "imu": t_imu,
    "battery": t_battery,
    "node_info": t_node_info,
    "ros2_status": t_ros2_status,
}


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        _LOG("selftest : bridge=%s" % BRIDGE_URL)
        _LOG(t_ros2_status({}).replace("\n", " | "))
        return
    _LOG("demarrage (bridge=%s)" % BRIDGE_URL)
    _rpc.serve(SERVER_NAME, TOOLS, HANDLERS)


if __name__ == "__main__":
    main()
