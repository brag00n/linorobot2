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

    from bamboo_interfaces.msg import NavState, RobotMode, TeleopState
    from bamboo_interfaces.srv import ChangeMode, NavCommand

    # services appelables par une etape : (type, fabrique de requete). Liste FERMEE : une etape ne peut
    # appeler que ce qui est declare ici, quoi que dise le fichier de missions.
    # Un service est « en marche » si l'un de ses noeuds temoins est vivant. Liste fermee, alignee sur docker/ondemand/groups.conf.
    SERVICE_NODES = {
        "bamboo_videotracking": ("/tracking_node", "/overlay_node", "/videotracking_container"),
        "bamboo_calibration": ("/bamboo_calibration",),
        "driver.lidarD500": ("/lidar_driver",),
        "navigation": ("/nav_supervisor", "/bt_navigator", "/slam_toolbox"),
    }
    PLAN = "/run/bamboo-ondemand/mission.plan"
    PLAN_STATE = "/run/bamboo-ondemand/mission.state"

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
            self.robotMode = ""
            self.navT = 0.0
            self.armed = False
            self.link = False
            self.guard, self.guardT = {}, 0.0
            self.linkT = 0.0
            self.batt = None
            self.reply = ""
            self.replyT = 0.0
            self._nodesCache, self._nodesT = [], 0.0
            self._lastBody, self._lastPubT = "", 0.0
            self.create_subscription(NavState, "/navigation/state", self._onNav, latched)
            self.create_subscription(RobotMode, "/bamboo/mode", self._onRobotMode, latched)
            self.create_subscription(TeleopState, "/teleop/state", self._onTeleop, latched)
            self.create_subscription(String, "/cc/guard/state", self._onGuard, latched)
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
        def _onRobotMode(self, m):
            self.robotMode = m.mode

        def _onNav(self, m):
            self.nav, self.navT = m, time.monotonic()

        def _onTeleop(self, m):
            self.armed = bool(m.armed)

        def _onGuard(self, m):
            """Etat du garde-fou, publie a 2 Hz. On lit ICI la liaison, pas sur /cc/guard/link : ce dernier n'est publie que
            lorsque sa valeur CHANGE, donc au bout de 6 s sans changement il paraissait perime et chaque mission qui roule etait
            refusee (« liaison absente ») alors que la liaison etait saine depuis des heures (2026-11-11)."""
            try:
                d = json.loads(m.data)
            except ValueError:
                return
            self.guard, self.guardT = d, time.monotonic()
            self.link = d.get("cc") == "ok" and d.get("video") in ("ok", "bad") and not d.get("tripped")
            self.linkT = self.guardT

        def _linkOk(self):
            # Frais = le garde-fou a publie dans les 3 dernieres secondes (il publie toutes les 0,5 s) : un garde-fou mort n'autorise rien.
            return bool(self.link) and time.monotonic() - self.linkT < 3.0

        def _runningServices(self):
            """Services en marche, deduits des noeuds ROS vivants (un conteneur = ses noeuds). Table fermee : seuls les services que la
            mission peut eteindre ou demarrer sont reconnus ; un noeud qui n'est dans aucune ligne ne dit rien."""
            nodes = set(self._nodes())
            return [svc for svc, marks in SERVICE_NODES.items() if any(m in nodes for m in marks)]

        def _state(self):
            """Etat a plat que le moteur sait lire (voir mission_core.check)."""
            s = {"noeuds": self._nodes(), "services": self._runningServices(), "robot.mode": self.robotMode}
            n = self.nav
            if n is not None and time.monotonic() - self.navT < 6.0:
                s.update({"nav.submode": n.submode, "nav.loc_state": n.loc_state,
                          "nav.ready": bool(n.ready), "nav.goal_status": n.goal_status,
                          "nav.auto_state": getattr(n, "auto_state", "off") or "off",
                          "nav.detail": getattr(n, "detail", "") or ""})
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
            if not ok:
                with self._lock:
                    self.engine.reset()                # un refus efface l'echec precedent : la page montre CE refus, pas l'ancien
            if ok:
                self._run(acts)                        # plans de services : eteindre le superflu, PUIS demarrer le necessaire
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
            if service == "/bamboo/change_mode":
                cli = self._client(ChangeMode, service)
                return cli.call_async(ChangeMode.Request(mode=str(args.get("mode", ""))))
            if service == "/bamboo/nav_stack":
                cli = self._client(SetBool, service)
                return cli.call_async(SetBool.Request(data=bool(args.get("data"))))
            return None

        def _client(self, typ, service):
            if service not in self._cli:
                self._cli[service] = self.create_client(typ, service, callback_group=self._cbg)
            return self._cli[service]

        def _plan(self, verb, names, why):
            """Depose un plan de services pour l'hote (ecriture atomique). Rend False si le dossier n'existe pas."""
            self._planId = getattr(self, "_planId", 0) + 1
            try:
                tmp = PLAN + ".tmp"
                with open(tmp, "w") as fh:
                    fh.write("id=%d\n%s %s\n" % (self._planId, verb, " ".join(names)))
                os.replace(tmp, PLAN)
            except OSError as e:
                self.get_logger().warn("plan de services non depose (%s) : %s" % (PLAN, e))
                return False
            self.get_logger().info("mission : %s -> %s %s" % (why, verb, " ".join(names)))
            return True

        def _run(self, acts):
            for a in acts:
                if "services" in a:
                    # Les plans partent UN A LA FOIS : l'hote doit avoir fini le precedent (eteindre avant de demarrer).
                    self._pendingPlans = getattr(self, "_pendingPlans", [])
                    self._pendingPlans.append(a)
                    continue
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

        def _servicesBusy(self):
            """Vrai tant qu'un plan de services est en file ou que l'hote n'a pas rapporte l'avoir termine."""
            if getattr(self, "_pendingPlans", []):
                return True
            w = getattr(self, "_planWait", 0)
            if not w:
                return False
            try:
                with open(PLAN_STATE) as fh:
                    m = fh.read().split()
                done = int(m[0].split("=")[1]) if m and m[0].startswith("id=") else 0
            except (OSError, ValueError, IndexError):
                done = 0
            if done >= w or time.monotonic() - self._planT > 120.0:
                self._planWait = 0
                return False
            return True

        def _pumpPlans(self):
            """Envoie le plan suivant quand l'hote a repondu au precedent (l'etat porte l'id du dernier plan traite)."""
            q = getattr(self, "_pendingPlans", [])
            if not q:
                return
            last = getattr(self, "_planId", 0)
            done = 0
            try:
                with open(PLAN_STATE) as fh:
                    m = fh.read().split()
                    done = int(m[0].split("=")[1]) if m and m[0].startswith("id=") else 0
                    self._planStatus = " ".join(m[1:])
            except (OSError, ValueError, IndexError):
                done = last if not os.path.exists(os.path.dirname(PLAN)) else done
            if getattr(self, "_planWait", 0) and done < self._planWait and time.monotonic() - self._planT < 120.0:
                return                                          # l'hote n'a pas fini le plan precedent
            a = q.pop(0)
            if self._plan(a["services"], a["names"], a.get("why", "")):
                self._planWait, self._planT = self._planId, time.monotonic()
            else:
                self._planWait = 0

        # ---- boucle ----
        def _tick(self):
            self._pumpPlans()
            if self._servicesBusy():
                self._publishState()                   # l'hote prepare les services : le moteur attend (pas d'etape, pas de delai)
                return
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
