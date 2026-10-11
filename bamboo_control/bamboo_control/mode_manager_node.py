#!/usr/bin/env python3
"""bamboo_mode_manager -- les MODES du robot (conduite, tracking, calibration, navigation...).

ETUDE ET CHOIX (2026-10-03, decision de l'utilisateur). Deux notions ROS 2 ont ete
examinees :
  * micro-ROS/system_modes : un mode y est un JEU DE VALEURS DE PARAMETRES par noeud, plus
    l'etat actif/inactif de chaque noeud, decrit dans un fichier modele ; un gestionnaire
    sert change_mode / get_mode / get_available_modes et publie la DEMANDE avant d'agir sur
    un sujet latche mode_request_info. Le depot n'est plus maintenu, n'est pas prevu pour la
    production, et suppose des noeuds LIFECYCLE -- aucun des notres ne l'est.
  * les arbres de comportement (BT) : faits pour ENCHAINER des comportements reactifs
    (Nav2 en est construit). Utiles DANS un mode (la sequence de calibration aujourd'hui,
    la navigation demain), pas pour CHOISIR le mode, qui reste une machine a etats pilotee
    par l'operateur.
D'ou ce noeud : les CONVENTIONS de system_modes, sans le paquet (memes services, memes
sujets, un fichier modele YAML), et des BT a l'interieur des modes (py_trees_ros, cf.
bamboo_base/calibration_node.py). Une migration vers system_modes reste possible.

CE QU'UN MODE PEUT FAIRE (config/modes.yaml) -- chaque champ est une CHAINE, items separes
par des ';' (une liste ROS vide ne se type pas, une chaine vide si) :
  label           texte court du HUD et de l'OLED
  requires        prerequis : `node:/nom` (noeud vivant), `param:nom` (booleen de CE noeud)
  enter_modecmd   ModeCmd a publier a l'entree, `cible=valeur` (groupe videotracking)
  exit_modecmd    idem a la sortie
  enter_params    `/noeud:param=valeur` poses a l'entree (valeurs de mode de system_modes)
  exit_params     idem a la sortie
  exit_cancel     actions dont les buts en cours sont ANNULES a la sortie (`/nom/action`) :
                  on ne quitte pas un mode qui fait tourner le robot sans l'avoir arrete

LA MANETTE NE TIENT PAS LE MODE : elle DEMANDE (croix gauche/droite -> change_mode
"__next__"/"__previous__") et LIT le mode applique sur /bamboo/mode. Un client MCP, un
script ou un futur superviseur NAV passent par les memes services : il n'y a qu'un
detenteur de l'etat.
"""
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

from bamboo_interfaces.msg import ModeCmd, RobotMode
from bamboo_interfaces.srv import ChangeMode, GetAvailableModes, GetMode
from bamboo_control.modes_model import FIELDS as _FIELDS, NEXT as _NEXT, PREV as _PREV
from bamboo_control.modes_model import parseModel


def _paramValue(text):
    """Valeur ROS typee depuis le texte du modele : bool, entier, reel, sinon chaine."""
    v = ParameterValue()
    low = text.strip().lower()
    if low in ("true", "false"):
        v.type, v.bool_value = ParameterType.PARAMETER_BOOL, low == "true"
        return v
    try:
        if "." not in low and "e" not in low:
            v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, int(low)
            return v
        v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, float(low)
        return v
    except ValueError:
        v.type, v.string_value = ParameterType.PARAMETER_STRING, text.strip()
        return v


_VALUE_FIELD = {ParameterType.PARAMETER_BOOL: "bool_value",
                ParameterType.PARAMETER_INTEGER: "integer_value",
                ParameterType.PARAMETER_DOUBLE: "double_value",
                ParameterType.PARAMETER_STRING: "string_value"}


def _sameValue(want, got):
    """Le noeud porte-t-il la valeur voulue ? (types scalaires du modele seulement)"""
    field = _VALUE_FIELD.get(want.type)
    return field is not None and got.type == want.type and \
        getattr(got, field) == getattr(want, field)


class ModeManager(Node):

    def __init__(self):
        super().__init__("bamboo_mode_manager")
        self._cbg = ReentrantCallbackGroup()
        names = list(self.declare_parameter("modes", ["conduite"]).value)
        default = str(self.declare_parameter("default_mode", names[0]).value)
        modeCmdTopic = self.declare_parameter(
            "mode_cmd_topic", "/videotracking/mode_cmd").value
        self._ackTimeout = float(self.declare_parameter("ack_timeout_s", 1.0).value)
        # Delai avant d'entrer dans le mode par defaut : laisser les noeuds du robot
        # apparaitre, sinon les reglages d'entree partiraient dans le vide.
        startDelay = float(self.declare_parameter("start_delay_s", 3.0).value)
        # Prerequis `param:` : booleens declares ICI, pousses par le launch (has_gimbal...).
        self._flags = {"has_gimbal": bool(self.declare_parameter("has_gimbal", False).value),
                       "has_lidar": bool(self.declare_parameter("has_lidar", False).value)}
        raw = {n: {f: self.declare_parameter("%s.%s" % (n, f), "").value for f in _FIELDS}
               for n in names}
        self._model = parseModel(names, raw)
        self._order = names
        if default not in self._model:
            raise ValueError("default_mode '%s' absent de `modes` %s" % (default, names))

        self._lock = threading.Lock()
        self._mode = None
        self._degraded = False
        self._detail = ""
        self._changes = 0

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pubMode = self.create_publisher(RobotMode, "/bamboo/mode", latched)
        self._pubReq = self.create_publisher(RobotMode, "/bamboo/mode_request_info", latched)
        self._pubModeCmd = self.create_publisher(ModeCmd, modeCmdTopic, latched)
        self._seq = 0
        self.create_service(ChangeMode, "/bamboo/change_mode", self._onChange,
                            callback_group=self._cbg)
        self.create_service(GetMode, "/bamboo/get_mode", self._onGet,
                            callback_group=self._cbg)
        self.create_service(GetAvailableModes, "/bamboo/get_available_modes",
                            self._onAvailable, callback_group=self._cbg)
        self._paramClients = {}
        self._getClients = {}
        self._cancelClients = {}
        # Noeuds ABSENTS lors de l'entree dans le mode courant : leurs reglages sont
        # REAPPLIQUES des qu'ils apparaissent (le groupe suivi demarre souvent apres nous).
        self._pending = set()
        self._pendingTimer = self.create_timer(3.0, self._applyPending,
                                               callback_group=self._cbg)

        self._diag = Updater(self)
        self._diag.setHardwareID("bamboo_control")
        self._diag.add("modes du robot", self._diagModes)

        self._default = default
        self._startTimer = self.create_timer(startDelay, self._enterDefault,
                                             callback_group=self._cbg)
        self.get_logger().info("gestionnaire de modes : %s (defaut %s)"
                               % (", ".join(names), default))

    # ------------------------------------------------------------ disponibilite
    def _nodeNames(self):
        return {(ns.rstrip("/") + "/" + n) for n, ns in self.get_node_names_and_namespaces()}

    def _why(self, name, nodes=None):
        """'' si le mode est disponible, sinon le premier prerequis manquant."""
        nodes = self._nodeNames() if nodes is None else nodes
        for req in self._model[name]["requires"]:
            if req.startswith("node:"):
                if req[5:] not in nodes:
                    return "noeud %s absent" % req[5:]
            elif not self._flags.get(req[6:], False):
                return "%s faux sur ce robot" % req[6:]
        return ""

    def _resolve(self, wanted):
        if wanted not in (_NEXT, _PREV):
            return wanted
        nodes = self._nodeNames()
        step = 1 if wanted == _NEXT else -1
        i = self._order.index(self._mode) if self._mode in self._order else 0
        for k in range(1, len(self._order)):
            cand = self._order[(i + step * k) % len(self._order)]
            if not self._why(cand, nodes):
                return cand
        return self._mode

    # ------------------------------------------------------------ services
    def _onGet(self, _req, resp):
        resp.mode = self._mode or ""
        resp.label = self._model[self._mode]["label"] if self._mode else ""
        resp.degraded = self._degraded
        return resp

    def _onAvailable(self, _req, resp):
        nodes = self._nodeNames()
        for n in self._order:
            why = self._why(n, nodes)
            resp.modes.append(n)
            resp.labels.append(self._model[n]["label"])
            resp.available.append(not why)
            resp.reasons.append(why)
        return resp

    def _onChange(self, req, resp):
        with self._lock:
            target = self._resolve(req.mode)
            if target not in self._model:
                resp.success, resp.mode = False, self._mode or ""
                resp.message = "mode inconnu : %s (connus : %s)" % (req.mode,
                                                                      ", ".join(self._order))
                return resp
            why = self._why(target)
            if why:
                resp.success, resp.mode = False, self._mode or ""
                resp.message = "%s indisponible : %s" % (target, why)
                return resp
            if target == self._mode:
                resp.success, resp.mode, resp.message = True, target, "deja actif"
                return resp
            ok, detail = self._switch(target)
            resp.success, resp.mode, resp.message = True, target, detail or "ok"
            if not ok:
                resp.message = "DEGRADE : " + detail
            return resp

    def _enterDefault(self):
        self._startTimer.cancel()
        with self._lock:
            if self._mode is None:
                why = self._why(self._default)
                if why:
                    self.get_logger().warn("mode par defaut %s indisponible (%s) : on y entre "
                                           "quand meme, c'est le mode de repli"
                                           % (self._default, why))
                self._switch(self._default)

    # ------------------------------------------------------------ bascule
    def _publish(self, pub, mode, previous, degraded=False, detail=""):
        m = RobotMode()
        m.header.stamp = self.get_clock().now().to_msg()
        m.mode, m.label = mode, self._model[mode]["label"]
        m.previous, m.degraded, m.detail = previous or "", degraded, detail
        pub.publish(m)

    def _switch(self, target):
        """Sortie du mode courant puis entree dans `target`. Appele SOUS self._lock."""
        previous = self._mode
        self._publish(self._pubReq, target, previous, detail="demande")
        faults = []
        if previous is not None:
            old = self._model[previous]
            for action in old["exit_cancel"]:
                faults += self._cancelAll(action)
            self._sendModeCmds(old["exit_modecmd"])
            faults += self._setParams(old["exit_params"])
        new = self._model[target]
        self._sendModeCmds(new["enter_modecmd"])
        faults += self._setParams(new["enter_params"])
        self._mode, self._degraded = target, bool(faults)
        self._detail = "; ".join(faults)
        self._changes += 1
        self._publish(self._pubMode, target, previous, self._degraded, self._detail)
        self.get_logger().info("mode %s -> %s%s" % (previous, target,
                                                     " (DEGRADE : %s)" % self._detail
                                                     if faults else ""))
        return not faults, self._detail

    def _sendModeCmds(self, cmds):
        for tgt, val in cmds:
            self._seq += 1
            m = ModeCmd()
            m.header.stamp = self.get_clock().now().to_msg()
            m.target, m.value, m.seq = tgt, val, self._seq
            self._pubModeCmd.publish(m)

    def _wait(self, future):
        ev = threading.Event()
        future.add_done_callback(lambda _f: ev.set())
        return ev.wait(self._ackTimeout)

    def _setParams(self, items):
        faults = []
        byNode = {}
        for node, prm, val in items:
            byNode.setdefault(node, []).append(Parameter(name=prm, value=_paramValue(val)))
        for node, params in byNode.items():
            cli = self._paramClients.get(node)
            if cli is None:
                cli = self.create_client(SetParameters, node + "/set_parameters",
                                         callback_group=self._cbg)
                self._paramClients[node] = cli
            if not cli.service_is_ready():
                # Noeud ABSENT : rien a regler, ce n'est pas une faute du mode (le groupe
                # suivi peut etre arrete en CONDUITE). Seul un REFUS degrade le mode. Le
                # reglage sera repris a son apparition (_applyPending).
                self.get_logger().info("mode : %s absent, reglage differe" % node)
                self._pending.add(node)
                continue
            self._pending.discard(node)
            fut = cli.call_async(SetParameters.Request(parameters=params))
            if not self._wait(fut) or fut.result() is None:
                faults.append("%s sans reponse" % node)
                continue
            # Un parametre INCONNU du noeud (version plus ancienne) est un refus : il se voit.
            for p, r in zip(params, fut.result().results):
                if not r.successful:
                    faults.append("%s:%s refuse (%s)" % (node, p.name, r.reason))
        return faults

    def _applyPending(self):
        """Reapplique les reglages d'entree du mode COURANT aux noeuds apparus depuis."""
        if self._mode is None:
            return
        self._checkDrift()
        if not self._pending:
            return
        with self._lock:
            nodes = self._nodeNames()
            ready = [n for n in self._pending if n in nodes]
            if not ready:
                return
            items = [it for it in self._model[self._mode]["enter_params"] if it[0] in ready]
            faults = self._setParams(items)
            if faults:
                self.get_logger().warn("mode %s : reglage differe refuse : %s"
                                       % (self._mode, "; ".join(faults)))
            else:
                self.get_logger().info("mode %s : reglages appliques a %s"
                                       % (self._mode, ", ".join(ready)))

    def _checkDrift(self):
        """RELIT les reglages du mode courant et reapplique ceux qui ont derive.

        Un noeud REDEMARRE (conteneur relance) revient avec ses valeurs PAR DEFAUT : relancer
        bamboo_videotracking en mode NAV rallumait la detection de visages et portait son CPU
        a 190 % (2026-10-11). Guetter sa DISPARITION du graphe ne marche pas : arrete
        brutalement, l'ancien participant DDS y reste jusqu'a expiration de son bail, le
        nouveau est deja la, et le nom ne quitte jamais la liste. On compare donc la VALEUR
        reelle a la valeur du mode -- ce qui couvre aussi un reglage change a la main.
        """
        byNode = {}
        for node, prm, val in self._model[self._mode]["enter_params"]:
            if node not in self._pending:
                byNode.setdefault(node, []).append((prm, val))
        for node, items in byNode.items():
            cli = self._getClients.get(node)
            if cli is None:
                cli = self.create_client(GetParameters, node + "/get_parameters",
                                         callback_group=self._cbg)
                self._getClients[node] = cli
            if not cli.service_is_ready():
                continue                       # absent : _setParams l'a mis en attente
            fut = cli.call_async(GetParameters.Request(names=[p for p, _ in items]))
            if not self._wait(fut) or fut.result() is None:
                continue
            drifted = [(node, p, v) for (p, v), got in zip(items, fut.result().values)
                       if not _sameValue(_paramValue(v), got)]
            if not drifted:
                continue
            with self._lock:
                faults = self._setParams(drifted)
            self.get_logger().warn(
                "mode %s : %s avait perdu %s (noeud redemarre ?) -> %s"
                % (self._mode, node, ", ".join(p for _, p, _ in drifted),
                   "reapplique" if not faults else "REFUS : " + "; ".join(faults)))

    def _cancelAll(self, action):
        """Annule TOUS les buts d'une action : goal_info nul = tous (action_msgs/CancelGoal)."""
        cli = self._cancelClients.get(action)
        if cli is None:
            cli = self.create_client(CancelGoal, action + "/_action/cancel_goal",
                                     callback_group=self._cbg)
            self._cancelClients[action] = cli
        if not cli.service_is_ready():
            return []                     # pas de serveur = rien a annuler
        fut = cli.call_async(CancelGoal.Request())
        if not self._wait(fut):
            return ["annulation de %s sans reponse" % action]
        return []

    # ------------------------------------------------------------ diagnostic
    def _diagModes(self, stat):
        nodes = self._nodeNames()
        stat.add("mode", str(self._mode))
        stat.add("changements", str(self._changes))
        for n in self._order:
            why = self._why(n, nodes)
            stat.add("dispo_" + n, "oui" if not why else "non (%s)" % why)
        if self._mode is None:
            stat.summary(DiagnosticStatus.WARN, "aucun mode applique (demarrage)")
        elif self._degraded:
            stat.summary(DiagnosticStatus.WARN, "%s DEGRADE : %s" % (self._mode, self._detail))
        else:
            stat.summary(DiagnosticStatus.OK, "mode %s" % self._model[self._mode]["label"])
        return stat


def main(argv=None):
    rclpy.init(args=argv)
    node = ModeManager()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
