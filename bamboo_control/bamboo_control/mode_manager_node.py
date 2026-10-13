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
import os
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from diagnostic_updater import Updater
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from std_srvs.srv import SetBool

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
        # Reglages CHANGES A LA MANETTE pendant le mode (ex. detection coupee en TRACKING) : le
        # controle de derive ne doit pas les « reparer » 3 s plus tard (2026-10-13 : la touche
        # detection de TRACKING etait annulee aussitot). Rendus au mode au prochain changement.
        self._userOwned = set()
        self.create_subscription(ModeCmd, modeCmdTopic, self._onModeCmd, latched,
                                 callback_group=self._cbg)
        self.create_service(ChangeMode, "/bamboo/change_mode", self._onChange,
                            callback_group=self._cbg)
        self.create_service(SetBool, "/bamboo/nav_stack", self._onNavStack,
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
        # --- services A LA DEMANDE (option B, 2026-10-11) ---
        # Le conteneur ne pilote pas Docker : il depose `on`/`off` dans <dir>/<groupe>.want ;
        # bamboo-ondemand.path (hote) l'execute sur une liste fermee, et repond dans
        # <groupe>.state. Sortir du mode n'arrete le groupe qu'apres `ondemand_off_delay_s` :
        # passer par CALIB ou SANTE ne relance pas 30 s de demarrage.
        self._odDir = str(self.declare_parameter("ondemand_dir", "/run/bamboo-ondemand").value)
        self._odDelay = float(self.declare_parameter("ondemand_off_delay_s", 120.0).value)
        self._odOffAt = {}            # groupe -> instant monotone de l'arret demande
        self._pendingTimer = self.create_timer(3.0, self._applyPending,
                                               callback_group=self._cbg)

        self._diag = Updater(self)
        self._diag.setHardwareID("bamboo_control")
        self._diag.add("modes du robot", self._diagModes)
        # Diagnostics TENANT LIEU des noeuds d'une pile A LA DEMANDE eteinte a dessein (cf.
        # _odDiagTick) : sans eux, l'arbre /diagnostics_agg les marquait PERIMES (rouge) des que
        # la pile NAV etait arretee -- c'est pourtant son etat normal (2026-10-13).
        self._pubDiag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0, self._odDiagTick, callback_group=self._cbg)

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
        ondemand = self._model[name]["ondemand"] and self._odUsable()
        for req in self._model[name]["requires"]:
            if req.startswith("node:"):
                # Mode a la demande : un noeud absent sera DEMARRE a l'entree.
                if req[5:] not in nodes and not ondemand:
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
            self._odAtStart()
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
        self._userOwned.clear()          # nouveau mode : ses reglages reprennent la main
        self._odOnSwitch(previous, target)
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
        self._odTick()
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

    # ------------------------------------------------------------ services a la demande
    def _odUsable(self):
        return os.path.isdir(self._odDir) and os.access(self._odDir, os.W_OK)

    def _odWant(self, group, on):
        """Depose la demande (ecriture ATOMIQUE : le .path de l'hote voit un fichier entier)."""
        if not self._odUsable():
            self.get_logger().warn("a la demande : %s absent (bamboo-ondemand.path non pose ?) "
                                   "-- %s non %s" % (self._odDir, group,
                                                     "demarre" if on else "arrete"))
            return
        tmp = os.path.join(self._odDir, ".%s.want.tmp" % group)
        with open(tmp, "w") as fh:
            fh.write("on\n" if on else "off\n")
        os.replace(tmp, os.path.join(self._odDir, "%s.want" % group))
        self.get_logger().info("a la demande : %s -> %s" % (group, "on" if on else "off"))

    def _odState(self, group):
        try:
            with open(os.path.join(self._odDir, "%s.state" % group)) as fh:
                return fh.read().rsplit(" ", 1)[0].strip()
        except OSError:
            return ""

    def _odOnSwitch(self, previous, target):
        # ENTRER dans le mode ne lance plus son groupe (2026-10-13) : c'est la manette qui le
        # demande (A long en NAV, service /bamboo/nav_stack) -- passer par NAV pour aller en
        # SANTE ne coute plus 30 s de demarrage ni la CPU de la pile. En SORTIR l'arrete
        # toujours apres le delai.
        # En SORTIR ne l'arrete plus non plus (demande utilisateur 2026-10-13) : la pile NAV
        # reste allumee dans les autres modes ; seul A long, de retour en NAV, l'eteint.
        new_g = self._model[target]["ondemand"]
        if new_g:
            self._odOffAt.pop(new_g, None)

    # Pour chaque groupe a la demande : (noeud, nom EXACT de son diagnostic) a remplacer
    # tant que le noeud est absent -- memes noms, donc memes cases de l'agregateur.
    OD_DIAGS = {"nav": (("/scan_monitor", "scan_monitor: lidar", "lidar"),
                        ("/nav_supervisor", "nav_supervisor: navigation", "navigation"))}

    def _odDiagTick(self):
        """Pile a la demande ETEINTE (ou en demarrage) : publie a la place de ses noeuds
        absents un diagnostic qui le DIT -- OK eteinte ou en demarrage, ERROR si le demarrage
        a echoue. Des que le vrai noeud est la, son propre diagnostic prend le relais."""
        if not self._odUsable():
            return
        nodes = None
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for g, items in self.OD_DIAGS.items():
            st = self._odState(g)
            for node, name, hwid in items:
                if nodes is None:
                    nodes = self._nodeNames()
                if node in nodes:
                    continue
                d = DiagnosticStatus(name=name, hardware_id=hwid)
                if st.startswith("error"):
                    d.level, d.message = DiagnosticStatus.ERROR, "demarrage a la demande en " \
                        "ECHEC (%s)" % st
                elif st.startswith(("starting", "running")):
                    d.level, d.message = DiagnosticStatus.OK, "demarrage a la demande en cours"
                else:
                    d.level = DiagnosticStatus.OK
                    d.message = "arrete (pile %s eteinte : A long en mode NAV)" % g
                d.values.append(KeyValue(key="a_la_demande", value=st or "aucun"))
                arr.status.append(d)
        if arr.status:
            self._pubDiag.publish(arr)

    def _onNavStack(self, req, resp):
        """Lancer (data=true) ou arreter le groupe A LA DEMANDE du mode courant (A long de la
        manette en NAV). Arret IMMEDIAT : la manette a deja fait sauver la carte."""
        groups = sorted({m["ondemand"] for m in self._model.values() if m["ondemand"]})
        g = self._model[self._mode]["ondemand"] if self._mode else ""
        g = g or (groups[0] if len(groups) == 1 else "")
        if not g:
            resp.success, resp.message = False, "aucune pile a la demande pour %s" % (
                self._mode or "-")
            return resp
        self._odOffAt.pop(g, None)
        self._odWant(g, bool(req.data))
        resp.success = True
        resp.message = "%s : %s demande" % (g, "demarrage" if req.data else "arret")
        return resp

    def _odTick(self):
        now = time.monotonic()
        cur = self._model[self._mode]["ondemand"] if self._mode else ""
        for g, t in list(self._odOffAt.items()):
            if g == cur:
                self._odOffAt.pop(g, None)
            elif now >= t:
                self._odOffAt.pop(g, None)
                self._odWant(g, False)

    def _odAtStart(self):
        """Un `on` reste d'une vie precedente : il est GARDE (2026-10-13 : la pile NAV ne
        s'arrete plus qu'a la demande, A long en NAV), et seulement signale."""
        for g in {m["ondemand"] for m in self._model.values() if m["ondemand"]}:
            try:
                with open(os.path.join(self._odDir, "%s.want" % g)) as fh:
                    if fh.read().strip() == "on":
                        self.get_logger().info("a la demande : %s reste allume (vie "
                                               "precedente)" % g)
            except OSError:
                pass

    # Cible ModeCmd envoyee par la MANETTE -> parametre qu'elle change sur son noeud.
    USER_TOGGLES = {"detect": ("/tracking_node", "detect_enabled")}

    def _onModeCmd(self, m):
        """Une commande de la manette sur un reglage du mode : il lui appartient jusqu'au
        prochain changement de mode. Seules les cibles connues (USER_TOGGLES) comptent ; le
        gestionnaire n'envoie lui-meme aucune de ces cibles."""
        owned = self.USER_TOGGLES.get(m.target)
        if not owned or self._mode is None or m.value not in ("on", "off"):
            return
        mode_val = next((v for n, p, v in self._model[self._mode]["enter_params"]
                         if (n, p) == owned), None)
        if mode_val is None:
            return
        # Seule une commande qui CONTREDIT le mode est un choix de l'utilisateur. La manette
        # renvoie aussi « detect off » a chaque sortie de TRACKING : la compter ferait perdre
        # la correction d'un noeud redemarre en NAV (detection rallumee, 190 % de CPU).
        asked = _paramValue("true" if m.value == "on" else "false")
        if _sameValue(_paramValue(mode_val), asked):
            self._userOwned.discard(owned)
        elif owned not in self._userOwned:
            self._userOwned.add(owned)
            self.get_logger().info("mode %s : %s:%s regle a la manette (%s), laisse tel quel"
                                   % (self._mode, owned[0], owned[1], m.value))

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
            if node not in self._pending and (node, prm) not in self._userOwned:
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
        for g in sorted({m["ondemand"] for m in self._model.values() if m["ondemand"]}):
            st = self._odState(g) or ("dossier absent" if not self._odUsable() else "aucun")
            if g in self._odOffAt:
                st += ", arret dans %.0f s" % max(0.0, self._odOffAt[g] - time.monotonic())
            stat.add("a_la_demande_" + g, st)
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
