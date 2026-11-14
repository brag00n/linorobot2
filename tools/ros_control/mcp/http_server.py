r"""http_server.py - Le serveur MCP `ros2-analysis` expose en streamable-HTTP, pour un client qui ne sait pas lancer un processus (Aria).

POURQUOI. Les serveurs MCP de ce depot sont des processus stdio lances par Claude Code. Aria tourne dans un conteneur Podman et n'a
que le streamable-HTTP : il lui faut une URL et un jeton. Cet adaptateur ne reecrit AUCUN outil : il sert les `TOOLS` et les `HANDLERS`
de `ros2.py`, donc ce que voit Aria est exactement ce que voit Claude Code, moins ce qui est retire ci-dessous.

SECURITE (decidee le 2026-11-14, a ne pas relacher sans en reparler) :
  - LECTURE SEULE PAR CONSTRUCTION : seuls les outils de `ros2.py` sont servis (jamais `ros2_action`, `orchestrator`, ni robot-action).
    `map` est retire : il ecrit un PNG sur le disque a un chemin choisi par l'appelant. Un test verifie qu'aucun handler servi ne publie.
  - JETON BEARER obligatoire, compare en temps constant. Il n'est JAMAIS dans le depot : variable ROS2_MCP_TOKEN, ou fichier hors depot
    (~/.ros2_mcp_token, cree au premier lancement). Sans jeton valide d'au moins 24 caracteres, le serveur ne demarre pas.
  - SOURCE : seules les adresses de ROS2_MCP_ALLOW sont servies (defaut : boucle locale et reseaux prives). Un conteneur Podman atteint le
    PC par une adresse privee, pas par 127.0.0.1 : on ecoute donc sur 0.0.0.0, et c'est le JETON qui protege, la liste est une 2e barriere.
  - Toute requete portant un en-tete Origin (= un navigateur) est refusee : un client MCP serveur n'en envoie pas (protection contre le
    rebond DNS recommandee par la specification MCP).
  - Corps limite a 64 Ko, fenetres d'ecoute bornees a 10 s, reponses tronquees a 100 000 caracteres.

PROTOCOLE : streamable-HTTP SANS ETAT. Un seul point d'entree, POST /mcp, JSON-RPC 2.0, reponse application/json. initialize,
notifications/* (202), ping, tools/list, tools/call. GET et DELETE renvoient 405 (pas de flux serveur, pas de session).

LANCEMENT (depuis tools/) :
    .venv/Scripts/python.exe -m ros_control.mcp.http_server
  ROS2_BRIDGE_URL  rosbridge du robot (defaut ws://127.0.0.1:9090 ; ici : wss://192.168.1.164:8443/rosbridge)
  ROS2_MCP_PORT    port (defaut 8790)         ROS2_MCP_HOST   adresse d'ecoute (defaut 0.0.0.0)
  ROS2_MCP_TOKEN / ROS2_MCP_TOKEN_FILE        jeton, ou fichier qui le contient
  ROS2_MCP_ALLOW   reseaux sources autorises, separes par des virgules (CIDR)
Depuis Podman : http://host.containers.internal:8790/mcp avec l'en-tete  Authorization: Bearer <jeton>.
"""
import hmac
import ipaddress
import json
import os
import secrets
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import _rpc

SERVER_NAME = "ros2-analysis-http"
MAX_BODY = 64 * 1024
MAX_WINDOW_S = 10.0
MAX_TEXT = 100_000
MIN_TOKEN = 24
DENY_TOOLS = {"map"}                       # ecrit un fichier a un chemin fourni par l'appelant
WINDOW_KEYS = ("timeout", "window_s", "seconds")
DEFAULT_ALLOW = "127.0.0.0/8,::1/128,10.0.0.0/8,172.16.0.0/12,192.168.0.0/16"
DEFAULT_PROTOCOL = _rpc.DEFAULT_PROTOCOL
_LOG = _rpc.make_log("ros2-mcp-http")


def load_token(env=None):
    """(jeton, origine). Variable d'abord, sinon fichier hors depot (cree au premier lancement). Jamais ecrit ailleurs."""
    env = os.environ if env is None else env
    tok = (env.get("ROS2_MCP_TOKEN") or "").strip()
    if tok:
        return tok, "variable ROS2_MCP_TOKEN"
    path = env.get("ROS2_MCP_TOKEN_FILE") or os.path.join(os.path.expanduser("~"), ".ros2_mcp_token")
    if os.path.exists(path):
        with open(path, encoding="utf-8") as fh:
            return fh.read().strip(), "fichier " + path
    tok = secrets.token_urlsafe(32)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(tok + "\n")
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass
    return tok, "fichier " + path + " (cree)"


def parse_nets(text):
    return [ipaddress.ip_network(x.strip(), strict=False) for x in (text or DEFAULT_ALLOW).split(",") if x.strip()]


def clamp_args(args):
    """Borne les durees d'ecoute : un appelant ne doit pas pouvoir bloquer un fil du serveur."""
    out = dict(args or {})
    for k in WINDOW_KEYS:
        v = out.get(k)
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            out[k] = max(0.1, min(float(v), MAX_WINDOW_S))
    return out


class McpHttp(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, addr, token, tools, handlers, nets, log=_LOG):
        if len(token or "") < MIN_TOKEN:
            raise ValueError("jeton trop court (%d caracteres, %d au moins)" % (len(token or ""), MIN_TOKEN))
        self.token = token.encode("utf-8")
        self.tools = [t for t in tools if t["name"] not in DENY_TOOLS and t["name"] in handlers]
        self.handlers = {t["name"]: handlers[t["name"]] for t in self.tools}
        self.nets = nets
        self.log = log
        super().__init__(addr, _Handler)

    def source_ok(self, ip):
        try:
            a = ipaddress.ip_address(ip.split("%")[0])
            if getattr(a, "ipv4_mapped", None):
                a = a.ipv4_mapped
        except ValueError:
            return False
        return any(a in n for n in self.nets)

    def dispatch(self, msg):
        """Un message JSON-RPC -> sa reponse, ou None pour une notification."""
        if not isinstance(msg, dict) or msg.get("jsonrpc") != "2.0":
            return {"jsonrpc": "2.0", "id": None, "error": {"code": -32600, "message": "requete JSON-RPC 2.0 invalide"}}
        mid, method = msg.get("id"), msg.get("method")
        if mid is None or (isinstance(method, str) and method.startswith("notifications/")):
            return None

        def err(code, text):
            return {"jsonrpc": "2.0", "id": mid, "error": {"code": code, "message": text}}

        def ok(result):
            return {"jsonrpc": "2.0", "id": mid, "result": result}

        if method == "initialize":
            pv = (msg.get("params") or {}).get("protocolVersion") or DEFAULT_PROTOCOL
            return ok({"protocolVersion": pv, "capabilities": {"tools": {"listChanged": False}},
                       "serverInfo": {"name": SERVER_NAME, "version": _rpc.SERVER_VERSION}})
        if method == "ping":
            return ok({})
        if method == "tools/list":
            return ok({"tools": self.tools})
        if method == "tools/call":
            p = msg.get("params") or {}
            name, args = p.get("name"), p.get("arguments") or {}
            if name not in self.handlers:
                return err(-32602, "outil inconnu ou non expose : %s" % name)
            if not isinstance(args, dict):
                return err(-32602, "arguments : objet attendu")
            self.log("outil", name)
            try:
                text, bad = str(self.handlers[name](clamp_args(args))), False
            except Exception as e:                                      # noqa: BLE001 - une panne d'outil n'est pas une panne du serveur
                text, bad = "erreur outil %s: %s" % (name, e), True
            if len(text) > MAX_TEXT:
                text = text[:MAX_TEXT] + "\n[... tronque : %d caracteres]" % len(text)
            return ok({"content": [{"type": "text", "text": text}], "isError": bad})
        return err(-32601, "methode inconnue: %s" % method)


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "ros2-mcp"

    def log_message(self, fmt, *a):                                       # le journal par defaut ecrirait la ligne de requete
        pass

    def _reply(self, code, body=None, headers=None):
        data = b"" if body is None else json.dumps(body, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(code)
        if body is not None:
            self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def _guard(self):
        """Source, Origin, jeton. Rend True si la requete peut etre traitee ; sinon la reponse est deja partie."""
        ip = self.client_address[0]
        if not self.server.source_ok(ip):
            self.server.log("refus : source", ip)
            self._reply(403, {"error": "source non autorisee"})
            return False
        if self.headers.get("Origin") is not None:
            self.server.log("refus : en-tete Origin (navigateur) depuis", ip)
            self._reply(403, {"error": "origine refusee"})
            return False
        auth = self.headers.get("Authorization", "")
        got = auth[7:].strip().encode("utf-8") if auth[:7].lower() == "bearer " else b""
        if not hmac.compare_digest(got, self.server.token):
            self.server.log("refus : jeton invalide depuis", ip)
            time.sleep(0.3)                                                # freine le sondage du jeton
            self._reply(401, {"error": "jeton requis"}, {"WWW-Authenticate": "Bearer"})
            return False
        return True

    def do_POST(self):
        if self.path.split("?")[0] != "/mcp":
            return self._reply(404, {"error": "introuvable"})
        if not self._guard():
            return
        try:
            n = int(self.headers.get("Content-Length", ""))
        except ValueError:
            return self._reply(411, {"error": "Content-Length requis"})
        if n < 0 or n > MAX_BODY:
            return self._reply(413, {"error": "corps trop gros (%d octets max)" % MAX_BODY})
        raw = self.rfile.read(n)
        try:
            msg = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return self._reply(400, {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "JSON invalide"}})
        if isinstance(msg, list):                                          # lot : accepte, comme le permet la version 2025-03-26
            out = [r for r in (self.server.dispatch(m) for m in msg) if r is not None]
            return self._reply(202) if not out else self._reply(200, out)
        out = self.server.dispatch(msg)
        return self._reply(202) if out is None else self._reply(200, out)

    def _no(self):
        self._reply(405, {"error": "POST /mcp seulement"}, {"Allow": "POST"})

    do_GET = do_DELETE = do_PUT = do_PATCH = _no


def make_server(host, port, token, tools, handlers, nets):
    return McpHttp((host, port), token, tools, handlers, nets)


def main():
    from . import ros2                                                      # import tardif : les tests n'ont pas besoin de rosbridge
    token, origin = load_token()
    if len(token) < MIN_TOKEN:
        sys.exit("jeton trop court dans %s (%d caracteres, %d au moins)" % (origin, len(token), MIN_TOKEN))
    host = os.environ.get("ROS2_MCP_HOST", "0.0.0.0")
    port = int(os.environ.get("ROS2_MCP_PORT", "8790"))
    nets = parse_nets(os.environ.get("ROS2_MCP_ALLOW"))
    srv = make_server(host, port, token, ros2.TOOLS, ros2.HANDLERS, nets)
    _LOG("ecoute sur http://%s:%d/mcp ; %d outils (lecture seule) ; rosbridge %s" % (host, port, len(srv.tools), ros2.BRIDGE_URL))
    _LOG("jeton : %s (jamais affiche) ; sources autorisees : %s" % (origin, ", ".join(str(n) for n in nets)))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        _LOG("arret")


if __name__ == "__main__":
    main()
