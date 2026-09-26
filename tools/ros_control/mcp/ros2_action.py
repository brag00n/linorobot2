#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""ros2_action.py - Serveur MCP (stdio) d'ACTUATION bornee du graphe ROS2, via rosbridge.

Cinquieme serveur MCP du projet, et le SEUL qui ecrive dans le graphe ROS. Il ne reinvente
aucune infra : il importe la classe `Bridge` de `ros2.py` (meme WebSocket, meme
`ROS2_BRIDGE_URL`) et n'ajoute que des outils.

POURQUOI UN SERVEUR SEPARE, et pas trois outils de plus dans `ros2-analysis` : la lecture
seule de `ros2-analysis` est une propriete qu'on veut pouvoir affirmer sans relire sa table
d'outils. Deux serveurs = deux lignes distinctes dans `.mcp.json`, donc une frontiere
visible dans la CONFIGURATION, pas seulement dans le code.

Outils :
  cmd_vel                -> publie un Twist borne pendant N s, ARRET garanti
  call_service           -> appel generique (save_map, lifecycle, set_parameters...)
  param_get / param_set  -> parametres d'un noeud (raccourcis typants sur call_service)

GARDE-FOUS, dans l'ordre ou ils s'appliquent :
  1. `cmd_vel` est DESARME par defaut : il refuse tant que ROS2_ACTION_ALLOW_CMD_VEL != "1".
     Motif materiel, pas ceremonial : sur les deux robots du projet les moteurs sont
     interdits (BambooWS a une attache moteur cassee ; bambooSTM32YB recoit 12,6 V bruts sur
     des moteurs 7,4 V nominaux). L'armement se fait dans `.mcp.json`, donc explicitement et
     de facon tracee -- jamais au detour d'un appel d'outil.
  2. Bornes DURES, mais jamais un refus : la consigne est ECRETEE et l'ecretage est DIT dans
     la reponse. Une valeur hors borne est presque toujours une faute de frappe, et un refus
     sec pousse a re-essayer plus fort.
  3. ARRET garanti : la republication vit dans `Bridge.publish_repeat`, qui envoie la
     consigne nulle dans un `finally`. Meme une exception au milieu de la rafale laisse le
     robot a l'arret.
  4. Ce serveur ne touche NI au port serie (mono-proprietaire, il appartient au driver) NI
     aux conteneurs (c'est `ros2-orchestrator`). Il ne parle qu'au graphe.

DEUX ARMEMENTS, ET C'EST VOULU : publier ici ne suffit pas, le driver a son propre verrou
`enable_cmd_vel` (defaut false) et ne cree meme pas la souscription tant qu'il est ferme.
Un `cmd_vel` "reussi" cote MCP sur un driver desarme ne fait donc RIEN -- d'ou la ligne de
rappel en fin de reponse : verifier l'effet par `wheel_rpm`/`odom`, pas par le succes du
publish (le protocole rosbridge n'acquitte pas un publish).

Lancement (via .mcp.json) :
    python -m ros_control.mcp.ros2_action
Test manuel (n'actionne rien) :
    python -m ros_control.mcp.ros2_action selftest
"""
import json
import os
import sys

from . import _rpc
from .ros2 import BRIDGE_URL, Bridge, BridgeError

SERVER_NAME = "ros2-action"
_LOG = _rpc.make_log("ros2-action")

# Bornes de consigne, identiques a celles de l'outil `cmd_vel` de robot-action : un chiffre
# lu dans un essai garde ainsi le meme sens quel que soit le chemin emprunte.
MAX_LINEAR = 0.4          # m/s
MAX_ANGULAR = 1.5         # rad/s
MAX_SECONDS = 5.0         # duree de rafale
PUBLISH_HZ = 10.0         # cadence de republication (watchdog driver : 0,5 s)

CMD_VEL_ARMED = os.environ.get("ROS2_ACTION_ALLOW_CMD_VEL", "0") == "1"


def _clamp(value, limit, name, notes):
    """Ecrete et CONSIGNE l'ecretage (cf. garde-fou 2)."""
    v = float(value)
    if v > limit:
        notes.append("%s ecrete de %.3f a %.3f" % (name, v, limit))
        return limit
    if v < -limit:
        notes.append("%s ecrete de %.3f a %.3f" % (name, v, -limit))
        return -limit
    return v


def _twist(lin, ang):
    return {"linear": {"x": lin, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": ang}}


def _call(service, args=None):
    return Bridge().call_service(service, args)


# ---------------------------------------------------------------------------
# Outils
# ---------------------------------------------------------------------------
def t_cmd_vel(args):
    if not CMD_VEL_ARMED:
        return ("REFUSE : l'actuation moteur est desarmee cote MCP. Les deux robots du "
                "projet ont leurs moteurs interdits (attache M1 cassee sur BambooWS ; PWM "
                "non mis a l'echelle sur bambooSTM32YB, pack 12,6 V sur moteurs 7,4 V). "
                "Pour armer : ROS2_ACTION_ALLOW_CMD_VEL=1 dans l'entree `ros2-action` du "
                ".mcp.json, puis relancer le client MCP. Et cote robot le driver a son "
                "PROPRE verrou : `enable_cmd_vel` (defaut false) -- sans lui la "
                "souscription /cmd_vel n'existe meme pas.")
    notes = []
    lin = _clamp(args.get("linear", 0.0), MAX_LINEAR, "linear.x", notes)
    ang = _clamp(args.get("angular", 0.0), MAX_ANGULAR, "angular.z", notes)
    secs = _clamp(args.get("seconds", 1.5), MAX_SECONDS, "seconds", notes)
    secs = max(0.1, abs(secs))
    topic = args.get("topic", "/cmd_vel")

    sent = Bridge().publish_repeat(topic, "geometry_msgs/msg/Twist", _twist(lin, ang),
                                   PUBLISH_HZ, secs, stop_msg=_twist(0.0, 0.0))
    lines = ["%s : linear.x=%.3f m/s angular.z=%.3f rad/s pendant %.1f s "
             "(%d trames a %.0f Hz), puis ARRET (3 trames nulles)."
             % (topic, lin, ang, secs, sent, PUBLISH_HZ)]
    lines += ["  " + n for n in notes]
    lines.append("  Verifier l'effet par `wheel_rpm` / `odom` (serveur ros2-analysis) : "
                 "rosbridge n'acquitte pas un publish, donc un envoi reussi ne prouve PAS "
                 "qu'un noeud ecoutait.")
    return "\n".join(lines)


def t_call_service(args):
    """Appel generique : c'est LUI qui debloque le reste (save_map, lifecycle, parametres)."""
    service = args.get("service")
    if not service:
        return "Parametre 'service' requis (ex. /slam_toolbox/save_map)."
    raw = args.get("args")
    if isinstance(raw, str) and raw.strip():
        try:
            raw = json.loads(raw)
        except Exception as exc:                                    # noqa: BLE001
            return "args n'est pas du JSON valide : %s" % exc
    if raw in (None, ""):
        raw = None
    return "%s -> %s" % (service,
                         json.dumps(_call(service, raw), indent=2, ensure_ascii=False))


# --- parametres : typage EXPLICITE, car rosbridge ne devine rien -------------
# rcl_interfaces/msg/ParameterType : 0 NOT_SET, 1 BOOL, 2 INTEGER, 3 DOUBLE, 4 STRING.
_PTYPE = {"bool": 1, "int": 2, "double": 3, "string": 4}
_PFIELD = {1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value"}


def _decode(pv):
    field = _PFIELD.get(pv.get("type", 0))
    if field is None:
        return "(type %s non gere : %s)" % (pv.get("type"), pv)
    return pv.get(field)


def t_param_get(args):
    node = args.get("node")
    names = args.get("names")
    if not node or not names:
        return "Parametres 'node' et 'names' requis (names : noms separes par des virgules)."
    if isinstance(names, str):
        names = [n.strip() for n in names.split(",") if n.strip()]
    res = _call("%s/get_parameters" % node.rstrip("/"), {"names": names})
    values = res.get("values") or []
    if len(values) != len(names):
        return ("reponse incoherente de %s : %d valeurs pour %d noms -> %s"
                % (node, len(values), len(names), json.dumps(res, ensure_ascii=False)))
    lines = ["%s :" % node]
    for name, pv in zip(names, values):
        if pv.get("type", 0) == 0:
            # PARAMETER_NOT_SET : le noeud existe mais ne declare pas ce nom. Ce n'est pas
            # une erreur de transport -- le dire evite de chercher une panne de rosbridge.
            lines.append("  %-28s (PARAMETER_NOT_SET : inconnu de ce noeud)" % name)
        else:
            lines.append("  %-28s %s" % (name, _decode(pv)))
    return "\n".join(lines)


def t_param_set(args):
    node = args.get("node")
    name = args.get("name")
    if not node or not name:
        return "Parametres 'node' et 'name' requis."
    if "value" not in args:
        return "Parametre 'value' requis."
    value = args["value"]
    given = args.get("type")
    if given:
        code = _PTYPE.get(str(given))
        if code is None:
            return "type inconnu : %s (attendu : %s)" % (given, ", ".join(sorted(_PTYPE)))
    else:
        # Inference DELIBEREMENT etroite. ROS distingue 2 (int) de 3 (double) et REFUSE la
        # confusion : `counts_per_rev` est declare en double cote driver, donc "1320" doit
        # partir en DOUBLE. D'ou la regle "un nombre sans indication est un double" ; pour
        # un vrai entier il faut ecrire type=int.
        txt = str(value).strip()
        if isinstance(value, bool) or txt.lower() in ("true", "false"):
            code = 1
        else:
            try:
                float(txt)
                code = 3
            except ValueError:
                code = 4

    if code == 1:
        value = value if isinstance(value, bool) else str(value).strip().lower() == "true"
    elif code == 2:
        value = int(float(value))
    elif code == 3:
        value = float(value)
    else:
        value = str(value)

    pv = {"type": code, "bool_value": False, "integer_value": 0,
          "double_value": 0.0, "string_value": ""}
    pv[_PFIELD[code]] = value
    res = _call("%s/set_parameters" % node.rstrip("/"),
                {"parameters": [{"name": name, "value": pv}]})
    results = res.get("results") or []
    if not results:
        return ("aucun resultat renvoye par %s/set_parameters : %s"
                % (node, json.dumps(res, ensure_ascii=False)))
    r = results[0]
    ok = bool(r.get("successful"))
    out = "%s %s = %r (type %d) -> %s" % (node, name, value, code,
                                          "ACCEPTE" if ok else "REFUSE")
    if r.get("reason"):
        out += " (%s)" % r["reason"]
    if ok:
        out += ("\n  Le callback du driver ecrit AUSSI dans la carte puis relit : confirmer "
                "par la relecture publiee (esp32_status / stm32_status), pas par cet "
                "ACCEPTE, qui ne dit que l'acceptation cote ROS.")
    return out


TOOLS = [
    {"name": "cmd_vel",
     "description": "Publie un Twist BORNE sur /cmd_vel pendant N secondes (republication "
                    "10 Hz) puis garantit l'ARRET. Bornes : +-0.4 m/s, +-1.5 rad/s, <=5 s ; "
                    "une valeur hors borne est ECRETEE, pas refusee. DESARME par defaut "
                    "(ROS2_ACTION_ALLOW_CMD_VEL=1 dans le .mcp.json) -- et le driver a son "
                    "propre verrou enable_cmd_vel.",
     "inputSchema": {"type": "object", "properties": {
         "linear": {"type": "number", "description": "linear.x en m/s (-0.4..0.4)"},
         "angular": {"type": "number", "description": "angular.z en rad/s (-1.5..1.5)"},
         "seconds": {"type": "number", "description": "duree de la rafale en s (0.1..5)"},
         "topic": {"type": "string", "description": "topic de consigne (defaut /cmd_vel)"}}}},
    {"name": "call_service",
     "description": "Appel de service ROS2 generique via rosbridge (set_parameters, "
                    "slam_toolbox/save_map, lifecycle...). args en objet JSON.",
     "inputSchema": {"type": "object", "properties": {
         "service": {"type": "string", "description": "nom complet du service"},
         "args": {"type": "string", "description": "arguments en JSON (objet), optionnel"}},
         "required": ["service"]}},
    {"name": "param_get",
     "description": "Lit des parametres d'un noeud (<node>/get_parameters). Un parametre "
                    "inconnu revient en PARAMETER_NOT_SET, pas en erreur de transport.",
     "inputSchema": {"type": "object", "properties": {
         "node": {"type": "string", "description": "noeud (ex. /esp32_driver)"},
         "names": {"type": "string", "description": "noms separes par des virgules"}},
         "required": ["node", "names"]}},
    {"name": "param_set",
     "description": "Ecrit UN parametre d'un noeud (<node>/set_parameters). Le type ROS est "
                    "infere (un nombre sans indication part en DOUBLE, ROS refusant la "
                    "confusion int/double) ou force par `type` (bool|int|double|string). Un "
                    "ACCEPTE ne prouve pas que la carte a pris la valeur : lire la "
                    "relecture publiee.",
     "inputSchema": {"type": "object", "properties": {
         "node": {"type": "string", "description": "noeud (ex. /esp32_driver)"},
         "name": {"type": "string", "description": "nom du parametre"},
         "value": {"type": "string", "description": "valeur"},
         "type": {"type": "string", "description": "bool|int|double|string (optionnel)"}},
         "required": ["node", "name", "value"]}},
]

HANDLERS = {
    "cmd_vel": t_cmd_vel,
    "call_service": t_call_service,
    "param_get": t_param_get,
    "param_set": t_param_set,
}


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "selftest":
        _LOG("selftest : bridge=%s cmd_vel arme=%s" % (BRIDGE_URL, CMD_VEL_ARMED))
        try:
            _LOG("type de /cmd_vel : %s"
                 % _call("/rosapi/topic_type", {"topic": "/cmd_vel"}))
        except BridgeError as exc:
            _LOG("bridge injoignable : %s" % exc)
        return
    _LOG("demarrage (bridge=%s, cmd_vel arme=%s)" % (BRIDGE_URL, CMD_VEL_ARMED))
    _rpc.serve(SERVER_NAME, TOOLS, HANDLERS)


if __name__ == "__main__":
    main()
