#!/usr/bin/env python3
"""bamboo_mission_manager -- execute les MISSIONS du Command Center (capacites enchainees).

Le moteur (mission_core.py) decide ; ce noeud ne fait que : lire l'etat du robot, appeler les services que
le moteur demande, et publier l'avancement. Il ne PILOTE aucun moteur et ne s'arme jamais : une etape qui
roule exige la traction deja armee par l'utilisateur et la liaison du CC saine (/cc/guard/link).

Interfaces (STANDARD, pour ne pas recompiler bamboo_interfaces) :
  /cc/mission/start   (std_srvs/Trigger-like via String)   voir ci-dessous
  /cc/mission/cancel  (std_srvs/Trigger)                   annule la mission en cours (ARRET)
  /cc/mission/state   (std_msgs/String, JSON, latche, 2 Hz) etat, etapes, raison, journal
  /cc/mission/list    (std_msgs/String, JSON, latche)       missions disponibles et leurs parametres

Demarrer : publier sur /cc/mission/request un String JSON {"mission": "Exploration", "params": {...}} ;
la reponse (acceptee ou refusee, avec la raison) est lisible dans /cc/mission/state (`reponse`). Un topic
plutot qu'un service : rosbridge n'a alors qu'un topic a ouvrir en ecriture, et la page n'attend pas un appel
bloquant pendant les ~30 s de demarrage de la pile NAV.
"""
import json
import os
import threading
import time

import yaml

from bamboo_control import mission_core as mc

STATE_HZ = 2.0


def loadModel(path):
    with open(path, "r", encoding="utf-8") as fh:
        return mc.validate(yaml.safe_load(fh))


def main(argv=None):
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from ament_index_python.packages import get_package_share_directory
    from std_msgs.msg import Bool, String
    from std_srvs.srv import SetBool, Trigger

    from bamboo_interfaces.msg import NavState, TeleopState
    from bamboo_interfaces.srv import NavCommand

    # services appelables par une etape : (type, fabrique de requete). Liste FERMEE : une etape ne peut
    # appeler que ce qui est declare ici, quoi que dise le fichier de missions.
    class MissionNode(Node):
        def __init__(self):
            super().__init__("bamboo_mission_manager")
            path = self.declare_parameter(
                "missions_file", os.path.join(get_package_share_directory("bamboo_control"),
                                              "config", "missions.yaml")).value
            self.model = loadModel(path)
            self.engine = mc.MissionEngine(self.model)
            self._lock = threading.RLock()
            self._cbg = ReentrantCallbackGroup()
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.nav = None
            self.navT = 0.0
            self.armed = False
            self.link = False
            self.linkT = 0.0
            self.batt = None
            self.reply = ""
            self.replyT = 0.0
            self._nodesCache, self._nodesT = [], 0.0
            self._lastBody, self._lastPubT = "", 0.0
            self.create_subscription(NavState, "/navigation/state", self._onNav, latched)
            self.create_subscription(TeleopState, "/teleop/state", self._onTeleop, latched)
            self.create_subscription(Bool, "/cc/guard/link", self._onLink, latched)
            self.create_subscription(String, "/cc/mission/request", self._onRequest, 10)
            self.create_subscription(String, "/cc/mission/cancel_req", self._onCancelReq, 10)
            self._pubState = self.create_publisher(String, "/cc/mission/state", latched)
            self._pubList = self.create_publisher(String, "/cc/mission/list", latched)
            self._cli = {}
            self.create_timer(1.0 / STATE_HZ, self._tick, callback_group=self._cbg)
            self._publishList()
            self.get_logger().info("bamboo_mission_manager : %d capacites, %d missions (%s)" % (
                len(self.model["capacites"]), len(self.model["missions"]), path))

        # ---- entrees ----
        def _onNav(self, m):
            self.nav, self.navT = m, time.monotonic()

        def _onTeleop(self, m):
            self.armed = bool(m.armed)

        def _onLink(self, m):
            self.link, self.linkT = bool(m.data), time.monotonic()

        def _linkOk(self):
            # La liaison doit etre FRAICHE : un /cc/guard/link perime (garde-fou mort) n'autorise rien.
            return self.link and time.monotonic() - self.linkT < 6.0

        def _state(self):
            """Etat a plat que le moteur sait lire (voir mission_core.check)."""
            s = {"noeuds": self._nodes()}
            n = self.nav
            if n is not None and time.monotonic() - self.navT < 6.0:
                s.update({"nav.submode": n.submode, "nav.loc_state": n.loc_state,
                          "nav.ready": bool(n.ready), "nav.goal_status": n.goal_status,
                          "nav.auto_state": getattr(n, "auto_state", "off") or "off"})
            return s

        def _nodes(self):
            """Noms des noeuds vivants. Interroger le graphe DDS coute cher (il l'etait a 2 Hz : mesure du 2026-11-11, charge
            du RPi doublee, battements du garde-fou en retard) : la reponse est gardee 5 s. Une mission attend de toute facon
            plusieurs secondes l'apparition d'un noeud, ce delai ne la ralentit pas de facon perceptible."""
            now = time.monotonic()
            if now - self._nodesT < 5.0:
                return self._nodesCache
            try:
                self._nodesCache = [("%s/%s" % (ns.rstrip("/"), nm)) for nm, ns in
                                    self.get_node_names_and_namespaces()]
            except Exception:                                  # noqa: BLE001
                self._nodesCache = []
            self._nodesT = now
            return self._nodesCache

        # ---- commandes ----
        def _onRequest(self, msg):
            try:
                d = json.loads(msg.data)
                name, params = str(d["mission"]), dict(d.get("params") or {})
            except (ValueError, KeyError, TypeError):
                self._answer(False, "requete illisible (attendu : {\"mission\": ..., \"params\": {...}})")
                return
            with self._lock:
                ok, why, acts = self.engine.start(name, params, time.monotonic(), self._state(),
                                                  self._linkOk(), self.armed)
            self._answer(ok, "mission %s lancee" % name if ok else why)
            self.get_logger().info("mission %s : %s" % (name, "LANCEE" if ok else "REFUSEE (%s)" % why))
            self._publishState(force=True)

        def _onCancelReq(self, _msg):
            with self._lock:
                acts = self.engine.cancel(time.monotonic(), "arret demande depuis le CC")
            self.get_logger().warn("mission annulee (arret demande)")
            self._run(acts)
            self._answer(True, "mission annulee")
            self._publishState(force=True)

        def _answer(self, ok, text):
            self.reply, self.replyT = ("OK : " if ok else "REFUS : ") + text, time.monotonic()

        # ---- execution des actions du moteur ----
        def _call(self, service, args):
            """Appelle un service de la liste fermee ; rend le Future ou None si inconnu."""
            if service == "/nav_supervisor/command":
                cli = self._client(NavCommand, service)
                return cli.call_async(NavCommand.Request(command=str(args.get("command", "")),
                                                         arg=str(args.get("arg", ""))))
            if service == "/bamboo/nav_stack":
                cli = self._client(SetBool, service)
                return cli.call_async(SetBool.Request(data=bool(args.get("data"))))
            return None

        def _client(self, typ, service):
            if service not in self._cli:
                self._cli[service] = self.create_client(typ, service, callback_group=self._cbg)
            return self._cli[service]

        def _run(self, acts):
            for a in acts:
                svc = a["call"]
                fut = self._call(svc, a["args"])
                if fut is None:
                    with self._lock:
                        self.engine.call_result(time.monotonic(), False,
                                                "service %s hors de la liste autorisee" % svc)
                    continue
                self.get_logger().info("mission : %s -> %s %s" % (a.get("why", ""), svc, a["args"]))
                fut.add_done_callback(lambda f, s=svc: self._done(f, s))

        def _done(self, fut, svc):
            try:
                r = fut.result()
                ok, msg = bool(getattr(r, "success", True)), str(getattr(r, "message", ""))
            except Exception as exc:                            # noqa: BLE001
                ok, msg = False, str(exc)
            if not ok:
                with self._lock:
                    self.engine.call_result(time.monotonic(), False, "%s : %s" % (svc, msg))
                self.get_logger().warn("mission : appel %s refuse (%s)" % (svc, msg))

        # ---- boucle ----
        def _tick(self):
            with self._lock:
                acts = self.engine.tick(time.monotonic(), self._state(), self._linkOk(), self.armed)
            self._run(acts)
            self._publishState()

        def _publishState(self, force=False):
            """Publie l'etat. Au repos (aucune mission), 1 fois par 2 s ; en mission ou sur un changement, a chaque tic."""
            with self._lock:
                snap = self.engine.snapshot(time.monotonic())
                snap["journal"] = [[t, s] for t, s in self.engine.log[-12:]]
            snap["reponse"] = self.reply if time.monotonic() - self.replyT < 30.0 else ""
            snap["liaison"] = self._linkOk()
            snap["armee"] = self.armed
            body = json.dumps(snap, ensure_ascii=False, separators=(",", ":"))
            now = time.monotonic()
            idle = snap["etat"] != "running"
            if not force and idle and body == self._lastBody and now - self._lastPubT < 2.0:
                return
            self._lastBody, self._lastPubT = body, now
            self._pubState.publish(String(data=body))

        def _publishList(self):
            out = []
            for name, m in self.model["missions"].items():
                plan = mc.compose(self.model, name)
                caps = self.model["capacites"]
                out.append({"id": name, "description": m.get("description", ""),
                            "parametres": m.get("parametres") or [], "etapes": plan,
                            "roule": any(caps[c].get("arme") for c in plan),
                            "detail": [caps[c].get("description", c) for c in plan]})
            self._pubList.publish(String(data=json.dumps(out, ensure_ascii=False, separators=(",", ":"))))

    rclpy.init(args=argv)
    node = MissionNode()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        with node._lock:
            acts = node.engine.cancel(time.monotonic(), "noeud arrete")
        node._run(acts)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
