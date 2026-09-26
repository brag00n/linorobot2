r"""_rpc.py - Boucle JSON-RPC 2.0 (stdio) partagee par les serveurs MCP *ROS* de ce depot.

COPIE MINIMALE, ET C'EST UN CHOIX EXPLICITE. Ce fichier est un extrait de
`linorobot2_hardware/tools/robot_control/mcp/_rpc.py`, reduit a ce que les serveurs ROS
utilisent REELLEMENT -- verifie par lecture : `ros2.py` et `orchestrator.py` n'appellent
que `make_log` et `serve`. Ce qui n'est PAS repris ici est la moitie *telemetrie de l'app
Windows* (`resolve_log_dir`, classe `Logs`) : elle lit `robot_control/logs/`, un dossier
qui n'existe pas dans ce depot et ne doit pas y etre invente.

La duplication est donc bornee (une boucle de protocole figee par MCP, pas de la logique
metier) et elle est achetee contre l'INDEPENDANCE DES DEUX DEPOTS : l'alternative etait un
import croise par PYTHONPATH, qui ferait echouer les serveurs ROS de ce depot des que le
depot firmware n'est pas cote a cote sur la machine.

stdout est reserve au JSON-RPC : tout message de service part sur stderr (make_log()).
"""
import json
import sys

SERVER_VERSION = "2.0.0"
DEFAULT_PROTOCOL = "2025-06-18"


def make_log(tag):
    """Fabrique une fonction de log prefixee vers stderr (stdout = JSON-RPC)."""
    def _log(*a):
        print("[%s]" % tag, *a, file=sys.stderr, flush=True)
    return _log


def _send(obj):
    data = json.dumps(obj, separators=(",", ":")).encode("utf-8") + b"\n"
    sys.stdout.buffer.write(data)                     # bytes -> pas de conversion \r\n
    sys.stdout.buffer.flush()


def serve(server_name, tools, handlers):
    """Boucle JSON-RPC MCP. handlers : {nom -> callable(args) -> texte}."""
    def result_text(mid, text, is_error=False):
        _send({"jsonrpc": "2.0", "id": mid,
               "result": {"content": [{"type": "text", "text": text}],
                          "isError": is_error}})

    for raw in sys.stdin.buffer:
        line = raw.decode("utf-8", "replace").strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except Exception:
            continue
        mid = msg.get("id")
        method = msg.get("method")
        if method == "initialize":
            pv = (msg.get("params") or {}).get("protocolVersion", DEFAULT_PROTOCOL)
            _send({"jsonrpc": "2.0", "id": mid,
                   "result": {"protocolVersion": pv,
                              "capabilities": {"tools": {}},
                              "serverInfo": {"name": server_name,
                                             "version": SERVER_VERSION}}})
        elif method == "notifications/initialized":
            pass                                       # notification : pas de reponse
        elif method == "ping":
            _send({"jsonrpc": "2.0", "id": mid, "result": {}})
        elif method == "tools/list":
            _send({"jsonrpc": "2.0", "id": mid, "result": {"tools": tools}})
        elif method == "tools/call":
            p = msg.get("params") or {}
            name = p.get("name")
            args = p.get("arguments") or {}
            handler = handlers.get(name)
            if handler is None:
                _send({"jsonrpc": "2.0", "id": mid,
                       "error": {"code": -32602, "message": "outil inconnu: %s" % name}})
                continue
            try:
                result_text(mid, handler(args))
            except Exception as e:
                result_text(mid, "erreur outil %s: %s" % (name, e), is_error=True)
        elif mid is not None:                          # requete de methode inconnue
            _send({"jsonrpc": "2.0", "id": mid,
                   "error": {"code": -32601, "message": "methode inconnue: %s" % method}})
        # sinon : notification inconnue -> ignore
