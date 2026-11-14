"""Tests de l'adaptateur HTTP (http_server.py), sans robot ni rosbridge : outils factices.

    cd tools && .venv/Scripts/python.exe -m unittest ros_control.mcp.test_http_server -v
"""
import http.client
import inspect
import json
import os
import tempfile
import threading
import unittest

from ros_control.mcp import http_server as hs

TOKEN = "t" * 40
TOOLS = [{"name": n, "description": n, "inputSchema": {"type": "object"}} for n in ("battery", "echo", "map", "nodes")]
SEEN = []
HANDLERS = {"battery": lambda a: "12,3 V", "echo": lambda a: (SEEN.append(a), "ok")[1], "map": lambda a: "ECRIT UN PNG",
            "nodes": lambda a: (_ for _ in ()).throw(RuntimeError("rosbridge absent"))}


class Base(unittest.TestCase):
    nets = None

    @classmethod
    def setUpClass(cls):
        cls.srv = hs.make_server("127.0.0.1", 0, TOKEN, TOOLS, HANDLERS, cls.nets or hs.parse_nets(None))
        cls.srv.log = lambda *a: None
        cls.port = cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def call(self, method="POST", path="/mcp", body=None, headers=None, token=TOKEN, raw=None):
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
        h = {"Content-Type": "application/json"}
        if token is not None:
            h["Authorization"] = "Bearer " + token
        h.update(headers or {})
        data = raw if raw is not None else (None if body is None else json.dumps(body))
        c.request(method, path, body=data, headers=h)
        r = c.getresponse()
        txt = r.read().decode("utf-8")
        c.close()
        return r.status, (json.loads(txt) if txt else None), r

    def rpc(self, method, params=None, mid=1):
        s, j, _ = self.call(body={"jsonrpc": "2.0", "id": mid, "method": method, "params": params or {}})
        return s, j


class Protocol(Base):
    def test_initialize_echoes_the_client_version(self):
        s, j = self.rpc("initialize", {"protocolVersion": "2025-03-26"})
        self.assertEqual(s, 200)
        self.assertEqual(j["result"]["protocolVersion"], "2025-03-26")
        self.assertIn("tools", j["result"]["capabilities"])

    def test_notification_gets_202_without_body(self):
        s, j, _ = self.call(body={"jsonrpc": "2.0", "method": "notifications/initialized"})
        self.assertEqual((s, j), (202, None))

    def test_tools_list_hides_map(self):
        s, j = self.rpc("tools/list")
        self.assertEqual(sorted(t["name"] for t in j["result"]["tools"]), ["battery", "echo", "nodes"])

    def test_tool_call_returns_text(self):
        s, j = self.rpc("tools/call", {"name": "battery", "arguments": {}})
        self.assertEqual(j["result"]["content"][0]["text"], "12,3 V")
        self.assertFalse(j["result"]["isError"])

    def test_a_failing_tool_is_an_error_result_not_a_crash(self):
        s, j = self.rpc("tools/call", {"name": "nodes", "arguments": {}})
        self.assertEqual(s, 200)
        self.assertTrue(j["result"]["isError"])
        self.assertIn("rosbridge absent", j["result"]["content"][0]["text"])
        self.assertEqual(self.rpc("ping")[1]["result"], {})                 # le serveur survit

    def test_unknown_and_denied_tools_are_refused(self):
        for name in ("map", "ros2_action", "cmd_vel", "../etc"):
            s, j = self.rpc("tools/call", {"name": name, "arguments": {}})
            self.assertEqual(j["error"]["code"], -32602, name)

    def test_unknown_method_and_bad_json(self):
        self.assertEqual(self.rpc("resources/list")[1]["error"]["code"], -32601)
        s, j, _ = self.call(raw="{pas du json")
        self.assertEqual((s, j["error"]["code"]), (400, -32700))

    def test_get_and_delete_are_405(self):
        for m in ("GET", "DELETE"):
            s, j, r = self.call(method=m)
            self.assertEqual(s, 405)
            self.assertEqual(r.getheader("Allow"), "POST")

    def test_other_paths_are_404(self):
        self.assertEqual(self.call(path="/")[0], 404)


class Security(Base):
    def test_no_token_or_wrong_token_is_401(self):
        body = {"jsonrpc": "2.0", "id": 1, "method": "tools/list"}
        for tok in (None, "", "x" * 40, TOKEN[:-1], TOKEN + "x"):
            s, _, r = self.call(body=body, token=tok)
            self.assertEqual(s, 401, repr(tok))
            self.assertEqual(r.getheader("WWW-Authenticate"), "Bearer")

    def test_token_is_not_accepted_in_a_query_string(self):
        s, _, _ = self.call(path="/mcp?token=" + TOKEN, body={"jsonrpc": "2.0", "id": 1, "method": "ping"}, token=None)
        self.assertEqual(s, 401)

    def test_a_browser_origin_is_refused_even_with_the_token(self):
        s, _, _ = self.call(body={"jsonrpc": "2.0", "id": 1, "method": "ping"}, headers={"Origin": "https://evil.example"})
        self.assertEqual(s, 403)

    def test_oversized_body_is_413(self):
        s, _, _ = self.call(raw="x" * (hs.MAX_BODY + 1))
        self.assertEqual(s, 413)

    def test_listening_windows_are_bounded(self):
        SEEN.clear()
        self.rpc("tools/call", {"name": "echo", "arguments": {"topic": "/battery", "timeout": 3600, "window_s": -5}})
        self.assertLessEqual(SEEN[0]["timeout"], hs.MAX_WINDOW_S)
        self.assertGreaterEqual(SEEN[0]["window_s"], 0.1)

    def test_token_too_short_refuses_to_start(self):
        with self.assertRaises(ValueError):
            hs.make_server("127.0.0.1", 0, "court", TOOLS, HANDLERS, hs.parse_nets(None))

    def test_source_filter(self):
        s = self.srv
        for ip in ("127.0.0.1", "192.168.1.50", "172.20.0.3", "10.0.2.2", "::ffff:192.168.1.5"):
            self.assertTrue(s.source_ok(ip), ip)
        for ip in ("8.8.8.8", "172.32.0.1", "1.2.3.4", "pas-une-ip"):
            self.assertFalse(s.source_ok(ip), ip)


class OnlyListedSources(Base):
    nets = hs.parse_nets("192.168.50.0/24")

    def test_loopback_is_refused_when_not_listed(self):
        s, _, _ = self.call(body={"jsonrpc": "2.0", "id": 1, "method": "ping"})
        self.assertEqual(s, 403)


class ReadOnly(unittest.TestCase):
    """Garde-fou de structure : ce qui est SERVI ne peut pas publier ni appeler un service."""

    def test_served_tools_are_the_read_ones_and_never_publish(self):
        from ros_control.mcp import ros2
        srv = hs.make_server("127.0.0.1", 0, TOKEN, ros2.TOOLS, ros2.HANDLERS, hs.parse_nets(None))
        try:
            names = {t["name"] for t in srv.tools}
            self.assertNotIn("map", names)
            self.assertTrue({"battery", "nodes", "nav_state", "diagnostics", "echo"} <= names)
            for n, fn in srv.handlers.items():
                src = inspect.getsource(fn)
                for bad in ("publish_burst", "\"advertise\"", "set_parameters", "cmd_vel", "subprocess", "open("):
                    self.assertNotIn(bad, src, "%s : %s" % (n, bad))
        finally:
            srv.server_close()

    def test_no_action_module_is_imported(self):
        src = inspect.getsource(hs)
        for bad in ("ros2_action", "orchestrator", "robot_control"):
            self.assertNotIn("import " + bad, src.replace("from . import " + bad, "import " + bad))
            self.assertNotIn("from . import " + bad, src)


class Token(unittest.TestCase):
    def test_env_then_file_then_created(self):
        self.assertEqual(hs.load_token({"ROS2_MCP_TOKEN": " abc "})[0], "abc")
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "tok")
            t1, o1 = hs.load_token({"ROS2_MCP_TOKEN_FILE": p})
            self.assertGreaterEqual(len(t1), hs.MIN_TOKEN)
            self.assertIn("cree", o1)
            self.assertEqual(hs.load_token({"ROS2_MCP_TOKEN_FILE": p})[0], t1)


if __name__ == "__main__":
    unittest.main()
