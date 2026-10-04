#!/usr/bin/env python3
"""bamboo_calibration -- mesure du GLISSEMENT EN ROTATION, l'IMU servant de reference.

Lance depuis le mode `calibration` de la manette (A long, traction armee), ou par l'action
/bamboo_calibration/calibrate (MCP, script). LE ROBOT TOURNE : au sol, de la place (arcs :
~2 m x 2 m), batterie chargee. Le resultat est un CANDIDAT : rien n'est ecrit dans le
fichier canonique (regle JOURNAL.md) ; on l'essaie en RAM (groupe `essai` du mode), puis
l'utilisateur le recopie sous `calib:` apres validation.

COMPORTEMENT EN ARBRE (py_trees), parce que c'est la forme d'un enchainement reactif avec
garde et repli -- cf. l'etude en tete de bamboo_control/mode_manager_node.py :

  Parallel(succes = la Sequence) --+-- GARDE (echec = abandon) : traction armee a la manette,
                                   |     batterie >= min_battery_v, gyro frais, duree bornee
                                   +-- Sequence
                                        Preconditions          driver, gyro, armement
                                        Selector IMU           repos | (recalibrer, repos)
                                        Bypass(on)             compensation neutralisee
                                        Paliers rotation       w_cmd x 2 sens
                                        [Paliers arcs]         v x w_cmd x 2 sens
                                        Calcul + qualite       table s, robot souleve, signe
                                        Ecriture               candidat + rapport JSON
  A TOUTE SORTIE (succes, echec, annulation) : STOP franc puis Bypass(off), hors de l'arbre.

L'annulation vient de la manette (STOP, desarmement, stick gauche, changement de mode) ou
du gestionnaire de modes (sortie du mode `calibration`) ; le chien de garde du driver
(0,25 s) arrete le robot si ce noeud meurt.

Les MESURES : le gyro (/imu/data, angular_velocity.z, signe aux conventions ROS par le
driver) et la vitesse de lacet VUE DES ROUES, brute (/motion_compensation/state.w_wheels).
Le driver est trouve par le publieur de ce dernier sujet : aucun nom de robot ici.
"""
import math
import os
import threading
import time

import py_trees
from py_trees.common import ParallelPolicy, Status

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from bamboo_interfaces.action import Calibrate
from bamboo_interfaces.msg import MotionCompensationState, TeleopState
from bamboo_base import calibration_math as cm
from bamboo_base.motion_compensation import PREFIX, SCRATCH_VARIANT, VARIANT_KEYS

STATE_TOPIC = "/motion_compensation/state"


def _yaw(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class Ctx:
    """Ce que partagent les feuilles de l'arbre et le noeud pendant UN but."""

    def __init__(self, node, goal):
        self.node, self.goal = node, goal
        self.t0 = time.monotonic()
        self.abort = ""
        self.done = 0
        self.line = "demarrage"
        self.imu = []          # rapports des mesures au repos
        self.steps = []
        self.table = None
        self.measured = None
        self.faults, self.warns, self.pairs = [], [], []
        self.candidate = ""
        self.report = ""
        self.lines = []        # rapport LISIBLE (carte MESURES du HUD), une ligne par fait

    def note(self, text):
        self.lines.append(text)
        self.node.publishReport(self.lines)


class Leaf(py_trees.behaviour.Behaviour):
    def __init__(self, name, ctx):
        super().__init__(name)
        self.ctx = ctx

    def say(self, text):
        self.ctx.line = "%s : %s" % (self.name, text)
        self.ctx.node.get_logger().info(self.ctx.line)

    def terminate(self, new_status):
        if new_status == Status.SUCCESS:
            self.ctx.done += 1


class Guard(Leaf):
    """Ne reussit JAMAIS : RUNNING tant que tout va bien, FAILURE (= abandon) sinon."""

    def update(self):
        why = self.ctx.node.guardViolation(self.ctx)
        if why:
            self.ctx.abort = why
            self.ctx.node.get_logger().error("calibration ABANDONNEE : %s" % why)
            return Status.FAILURE
        return Status.RUNNING

    def terminate(self, new_status):
        pass


class Preconditions(Leaf):
    def update(self):
        n = self.ctx.node
        why = n.preconditions(self.ctx.goal.imu_check_only)
        if why:
            self.ctx.abort = why
            self.say("REFUS : " + why)
            return Status.FAILURE
        self.say("driver %s, gyro present" % n.driverName)
        return Status.SUCCESS


class RestImu(Leaf):
    def __init__(self, name, ctx, duration):
        super().__init__(name, ctx)
        self.duration = duration

    def initialise(self):
        self.t0 = time.monotonic()
        self.gz, self.yaw, self.prev, self.unwrapped = [], [], None, 0.0
        self.say("robot IMMOBILE, %.0f s" % self.duration)

    def update(self):
        n = self.ctx.node
        gz, yaw = n.freshGz(), n.lastYaw
        if gz is not None:
            self.gz.append(gz)
        if yaw is not None:
            if self.prev is not None:
                d = (yaw - self.prev + 180.0) % 360.0 - 180.0
                self.unwrapped += d
            self.prev = yaw
            self.yaw.append(self.unwrapped)
        if time.monotonic() - self.t0 < self.duration:
            return Status.RUNNING
        st = cm.rest_stats(self.gz, self.yaw, self.duration)
        ok, msg = cm.rest_ok(st, n.p["imu_max_bias"], n.p["imu_max_drift_deg"])
        st["ok"], st["verdict"] = ok, msg
        self.ctx.imu.append(st)
        self.ctx.note("IMU %s biais %.4f derive %.2f deg" % (
            "OK" if ok else "HORS SEUIL", st["biais_gz"], st["derive_cap_deg"]))
        self.say(("OK " if ok else "HORS SEUIL ") + msg)
        return Status.SUCCESS if ok else Status.FAILURE


class Recalibrate(Leaf):
    def initialise(self):
        self.t0 = time.monotonic()
        self.fut = self.ctx.node.callRecalibrate()
        self.say("biais gyro remesure par la carte, robot IMMOBILE")

    def update(self):
        if self.fut is None:
            return Status.FAILURE
        if time.monotonic() - self.t0 < self.ctx.node.p["recalibrate_settle_s"]:
            return Status.RUNNING
        return Status.SUCCESS if self.fut.done() else Status.FAILURE


class Bypass(Leaf):
    def __init__(self, name, ctx, on):
        super().__init__(name, ctx)
        self.on = on

    def initialise(self):
        self.fut = self.ctx.node.setDriver([("motion_compensation_bypass", self.on)])
        self.t0 = time.monotonic()

    def update(self):
        if self.fut is None:
            return Status.FAILURE
        if not self.fut.done():
            return Status.RUNNING if time.monotonic() - self.t0 < 2.0 else Status.FAILURE
        ok = self.fut.result() is not None and self.fut.result().result.successful
        self.say("compensation %s" % ("NEUTRALISEE" if self.on else "retablie"))
        return Status.SUCCESS if ok else Status.FAILURE


class Step(Leaf):
    """Un palier a consigne constante : mise en regime, mesure, arret, pause."""

    def __init__(self, name, ctx, v, w, measure_s=None):
        super().__init__(name, ctx)
        self.v, self.w = v, w
        self.measure_s = measure_s     # None = measure_s commun (rotations)

    def initialise(self):
        self.t0 = time.monotonic()
        self.gz, self.ww, self.vw = [], [], []
        self.stopped = False
        self.say("v %.2f m/s, w %.2f rad/s" % (self.v, self.w))

    def update(self):
        n, p = self.ctx.node, self.ctx.node.p
        t = time.monotonic() - self.t0
        run = p["settle_s"] + (self.measure_s if self.measure_s is not None else p["measure_s"])
        if t < run:
            n.drive(self.v, self.w)
            if t >= p["settle_s"]:
                gz, st = n.freshGz(), n.lastState
                if gz is not None and st is not None:
                    self.gz.append(gz)
                    self.ww.append(st.w_wheels)
                    self.vw.append(st.v_wheels)
            return Status.RUNNING
        if not self.stopped:
            n.stop()
            self.stopped = True
        if t < run + p["pause_s"]:
            return Status.RUNNING
        st = cm.step_stats(self.v, self.w, self.gz, self.ww, self.vw)
        self.ctx.steps.append(st)
        self.ctx.note("%-14s imu %+.2f roues %+.2f s %s" % (
            self.name, st["w_imu"], st["w_roues"],
            "--" if st["s"] is None else "%.2f" % st["s"]))
        self.say("w_imu %.3f, w_roues %.3f -> s %s" % (
            st["w_imu"], st["w_roues"], "?" if st["s"] is None else "%.3f" % st["s"]))
        return Status.SUCCESS


class Compute(Leaf):
    def update(self):
        c, p = self.ctx, self.ctx.node.p
        faults, warns = cm.check_steps(c.steps, p["max_dispersion"])
        pairs, asym = cm.asymmetry(c.steps, p["max_asymmetry"])
        c.pairs, c.faults, c.warns = pairs, faults + asym, warns
        c.table, c.measured = cm.build_table(c.steps, p["v_bins_mps"], p["w_bins_rps"],
                                             p["slip_max"], p["fill_unmeasured_rows"])
        for w in warns:
            self.ctx.node.get_logger().warn("calibration : " + w)
        if c.faults:
            self.say("REJET : " + " ; ".join(c.faults))
            for f in c.faults:
                c.note("REJET " + f[:48])
            return Status.FAILURE
        self.say("table s = %s" % c.table)
        c.note("TABLE s " + " ".join("%.2f" % x for x in c.table))
        return Status.SUCCESS


class Write(Leaf):
    def update(self):
        self.ctx.node.writeCandidate(self.ctx)
        self.say("candidat ecrit : %s" % self.ctx.candidate)
        return Status.SUCCESS


class CalibrationNode(Node):

    def __init__(self):
        super().__init__("bamboo_calibration")
        self._cbg = ReentrantCallbackGroup()
        d = self.declare_parameter
        self.p = {
            "rate_hz": float(d("rate_hz", 10.0).value),
            "min_battery_v": float(d("min_battery_v", 11.4).value),
            "imu_rest_s": float(d("imu_rest_s", 10.0).value),
            "imu_max_bias": float(d("imu_max_bias", 0.005).value),
            "imu_max_drift_deg": float(d("imu_max_drift_deg", 0.5).value),
            "recalibrate_settle_s": float(d("recalibrate_settle_s", 4.0).value),
            "gyro_max_age_s": float(d("gyro_max_age_s", 0.3).value),
            "spin_w_cmd": [float(x) for x in d("spin_w_cmd", [0.6, 1.2, 2.0, 3.0]).value],
            "arc_v": [float(x) for x in d("arc_v", [0.2, 0.4]).value],
            "arc_w_cmd": [float(x) for x in d("arc_w_cmd", [0.6, 1.5]).value],
            "settle_s": float(d("settle_s", 1.0).value),
            "measure_s": float(d("measure_s", 3.0).value),
            # Arcs plus COURTS que les rotations : ils deplacent le robot (0,4 m/s x 3 s =
            # 1,6 m par palier, trop pour une piece -- essai du 2026-10-04 arrete faute de
            # place).
            "arc_measure_s": float(d("arc_measure_s", 2.0).value),
            "pause_s": float(d("pause_s", 1.5).value),
            "max_duration_s": float(d("max_duration_s", 240.0).value),
            "v_bins_mps": [float(x) for x in d("v_bins_mps", [0.0, 0.2, 0.4]).value],
            "w_bins_rps": [float(x) for x in d("w_bins_rps", [0.5, 1.0, 1.5]).value],
            "slip_max": float(d("slip_max", 0.8).value),
            "max_dispersion": float(d("max_dispersion", 0.10).value),
            "max_asymmetry": float(d("max_asymmetry", 0.15).value),
            "fill_unmeasured_rows": str(d("fill_unmeasured_rows", "copy").value),
            "require_teleop_armed": bool(d("require_teleop_armed", True).value),
            "pi_test_kp": float(d("pi_test_kp", 0.5).value),
            "pi_test_ki": float(d("pi_test_ki", 0.0).value),
            "output_dir": str(d("output_dir", "").value),
        }
        self.robot = str(d("robot", "").value)
        # Lus dans le bloc /** du fichier canonique, comme le driver.
        self.wheelSep = float(d("wheel_separation_m", 0.0).value)
        self.fileVariant = str(d("hardware_variant", "origine").value)
        self.fileValues = {k: d("%s.%s.%s" % (PREFIX, self.fileVariant, k), v).value
                           for k, v in VARIANT_KEYS.items()}

        self.lastImu = self.lastYaw = None
        self.lastImuT = 0.0
        self.lastState = None
        self.lastBattery = None
        self.lastTeleop = None
        self.driverName = ""
        self._paramCli = {}
        self._lock = threading.Lock()
        self._busy = False
        self._last = None             # dernier calcul REUSSI : {variant, table, ...}
        # La variante `essai` est-elle ACTIVE dans le driver (candidat ou PI en RAM) ? Tenu
        # ici : l'etat du driver n'est plus ecoute au repos (cf. _listen).
        self._ramActive = False

        # Abonnements RAPIDES (IMU et etat de compensation, 30 Hz) crees PENDANT un but
        # seulement : au repos, le MultiThreadedExecutor de rclpy brulait 26 % d'un coeur
        # a les recevoir pour rien (mesure 2026-10-04, RPi deja sature). Cf. _listen().
        self._fastSubs = []
        self.create_subscription(BatteryState, "/battery", self._onBattery, 10,
                                 callback_group=self._cbg)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TeleopState, "/teleop/state", self._onTeleop, latched,
                                 callback_group=self._cbg)
        self.pubVel = self.create_publisher(Twist, "cmd_vel", 10)
        # Rapport LISIBLE pour le HUD (carte MESURES du mode CALIB) : latche, une ligne par
        # fait mesure, le dernier but seulement.
        self.pubReport = self.create_publisher(String, "~/report", latched)

        ActionServer(self, Calibrate, "~/calibrate", execute_callback=self._execute,
                     goal_callback=self._onGoal, cancel_callback=lambda _g: CancelResponse.ACCEPT,
                     callback_group=self._cbg)
        self.create_service(Trigger, "~/apply_candidate", self._onApply, callback_group=self._cbg)
        self.create_service(Trigger, "~/revert", self._onRevert, callback_group=self._cbg)
        self.create_service(SetBool, "~/set_pi", self._onSetPi, callback_group=self._cbg)
        self.get_logger().info("calibration prete (%s, variante fichier %s)"
                               % (self.robot or "?", self.fileVariant))

    # ------------------------------------------------------------ entrees
    def _onImu(self, m):
        self.lastYaw = _yaw(m.orientation)
        if m.angular_velocity_covariance[0] >= 0.0:
            self.lastImu = m.angular_velocity.z
            self.lastImuT = time.monotonic()

    def _onState(self, m):
        self.lastState = m

    def _onBattery(self, m):
        self.lastBattery = m.voltage

    def _onTeleop(self, m):
        self.lastTeleop = m

    def _listen(self, on):
        """Cree ou detruit les abonnements rapides. Attend un premier echantillon."""
        if on and not self._fastSubs:
            self._fastSubs = [
                self.create_subscription(Imu, "/imu/data", self._onImu,
                                         qos_profile_sensor_data, callback_group=self._cbg),
                self.create_subscription(MotionCompensationState, STATE_TOPIC, self._onState,
                                         10, callback_group=self._cbg)]
            t = time.monotonic()
            while (self.freshGz() is None or self.lastState is None) and (
                    time.monotonic() - t < 3.0):
                time.sleep(0.05)
        elif not on:
            for sub in self._fastSubs:
                self.destroy_subscription(sub)
            self._fastSubs = []
            self.lastImu = self.lastState = None

    def freshGz(self):
        if self.lastImu is None or time.monotonic() - self.lastImuT > self.p["gyro_max_age_s"]:
            return None
        return self.lastImu

    # ------------------------------------------------------------ driver
    def _findDriver(self):
        infos = self.get_publishers_info_by_topic(STATE_TOPIC)
        if not infos:
            return ""
        i = infos[0]
        return (i.node_namespace.rstrip("/") + "/" + i.node_name)

    def setDriver(self, items):
        """set_parameters_ATOMICALLY asynchrone sur le driver ; None si le service est absent.

        Atomique et non set_parameters : ce dernier applique les valeurs UNE PAR UNE, donc
        basculer hardware_variant et poser la table de la variante passerait par des etats
        intermediaires que le driver appliquerait (constate a l'essai : quatre rechargements
        pour un seul geste)."""
        name = self.driverName or self._findDriver()
        if not name:
            return None
        cli = self._paramCli.get(name)
        if cli is None:
            cli = self.create_client(SetParametersAtomically,
                                     name + "/set_parameters_atomically",
                                     callback_group=self._cbg)
            self._paramCli[name] = cli
        if not cli.wait_for_service(timeout_sec=1.0):
            return None
        req = SetParametersAtomically.Request(
            parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in items])
        return cli.call_async(req)

    def _setDriverSync(self, items, timeout=2.0):
        fut = self.setDriver(items)
        if fut is None:
            return False, "driver introuvable (aucun publieur de %s)" % STATE_TOPIC
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout) or fut.result() is None:
            return False, "driver sans reponse"
        r = fut.result().result
        return r.successful, (r.reason or "ok")

    def callRecalibrate(self):
        name = self.driverName or self._findDriver()
        if not name:
            return None
        cli = self.create_client(Trigger, name + "/recalibrate_imu", callback_group=self._cbg)
        if not cli.wait_for_service(timeout_sec=1.0):
            return None
        return cli.call_async(Trigger.Request())

    def publishReport(self, lines):
        self.pubReport.publish(String(data="\n".join(lines)))

    def drive(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pubVel.publish(t)

    def stop(self):
        for _ in range(3):
            self.drive(0.0, 0.0)

    # ------------------------------------------------------------ garde
    def preconditions(self, imu_only):
        self.driverName = self._findDriver()
        if not self.driverName:
            return "aucun driver ne publie %s" % STATE_TOPIC
        if self.freshGz() is None:
            return "pas de gyro frais sur /imu/data (couche hote trop ancienne ?)"
        if imu_only:
            return ""
        st = self.lastState
        if st is None or not st.armed:
            return "traction du driver DESARMEE (enable_cmd_vel=false)"
        if self.p["require_teleop_armed"] and not (self.lastTeleop and self.lastTeleop.armed):
            return "traction non armee a la manette (appui long sur B)"
        if self.lastBattery is not None and self.lastBattery < self.p["min_battery_v"]:
            return "batterie %.2f V < %.2f V : la recharger" % (self.lastBattery,
                                                               self.p["min_battery_v"])
        if self.wheelSep <= 0.0:
            return "wheel_separation_m absent du fichier canonique"
        return ""

    def guardViolation(self, ctx):
        if time.monotonic() - ctx.t0 > self.p["max_duration_s"]:
            return "duree maximale %.0f s depassee" % self.p["max_duration_s"]
        if self.freshGz() is None:
            return "gyro perdu"
        if ctx.goal.imu_check_only:
            return ""
        if self.p["require_teleop_armed"] and self.lastTeleop is not None \
                and not self.lastTeleop.armed:
            return "traction desarmee a la manette"
        if self.lastBattery is not None and self.lastBattery < self.p["min_battery_v"] - 0.5:
            return "batterie effondree (%.2f V)" % self.lastBattery
        return ""

    # ------------------------------------------------------------ arbre
    def _tree(self, ctx):
        g = ctx.goal
        imu = py_trees.composites.Selector("IMU", memory=True, children=[
            RestImu("IMU repos", ctx, self.p["imu_rest_s"]),
            py_trees.composites.Sequence("IMU recalibree", memory=True, children=[
                Recalibrate("recalibration gyro", ctx),
                RestImu("IMU repos (2)", ctx, self.p["imu_rest_s"])])])
        seq = [Preconditions("preconditions", ctx), imu]
        if not g.imu_check_only:
            seq.append(Bypass("bypass on", ctx, True))
            for w in self.p["spin_w_cmd"]:
                for sgn in (1.0, -1.0):
                    seq.append(Step("rotation %+.1f" % (sgn * w), ctx, 0.0, sgn * w))
            if g.include_arcs:
                # ALLER-RETOUR : (v, +w) puis (-v, -w) refait le MEME chemin en marche
                # arriere et ramene le robot pres de son depart ; puis (v, -w), (-v, +w).
                # Les quatre signes sont mesures et le robot reste a peu pres sur place
                # (demande utilisateur 2026-10-04, apres un essai qui manquait de place).
                am = self.p["arc_measure_s"]
                for v in self.p["arc_v"]:
                    for w in self.p["arc_w_cmd"]:
                        for sv, sw in ((1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0)):
                            seq.append(Step("arc v%+.1f w%+.1f" % (sv * v, sw * w), ctx,
                                            sv * v, sw * w, am))
            seq += [Bypass("bypass off", ctx, False), Compute("calcul", ctx), Write("ecriture", ctx)]
        main = py_trees.composites.Sequence("calibration", memory=True, children=seq)
        root = py_trees.composites.Parallel(
            "garde", policy=ParallelPolicy.SuccessOnSelected(children=[main]),
            children=[Guard("garde", ctx), main])
        # total = etapes de la Sequence (le Selector IMU n'en compte qu'une, cas nominal).
        return root, len(seq)

    # ------------------------------------------------------------ action
    def _onGoal(self, _goal):
        with self._lock:
            if self._busy:
                self.get_logger().warn("calibration deja en cours : but refuse")
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, gh):
        goal = gh.request
        ctx = Ctx(self, goal)
        root, total = self._tree(ctx)
        bt = py_trees.trees.BehaviourTree(root)
        period = 1.0 / self.p["rate_hz"]
        self.get_logger().warn("calibration %s (variante %s, arcs %s)" % (
            "IMU SEULE" if goal.imu_check_only else "COMPLETE", goal.variant or "essai",
            goal.include_arcs))
        lastLine = None
        self._listen(True)
        ctx.note("%s variante %s%s" % ("IMU seule," if goal.imu_check_only else "complete,",
                                       goal.variant or SCRATCH_VARIANT,
                                       ", arcs" if goal.include_arcs else ""))
        try:
            while rclpy.ok():
                if gh.is_cancel_requested:
                    ctx.abort = ctx.abort or "annulee"
                    root.stop(Status.INVALID)
                    break
                bt.tick()
                if ctx.line != lastLine:
                    lastLine = ctx.line
                    fb = Calibrate.Feedback(line=ctx.line, step=ctx.done, total=total)
                    gh.publish_feedback(fb)
                if root.status in (Status.SUCCESS, Status.FAILURE):
                    break
                time.sleep(period)
        finally:
            # HORS de l'arbre, inconditionnel : un arbre interrompu n'executerait pas ses
            # dernieres feuilles.
            self.stop()
            if not goal.imu_check_only:
                ok, why = self._setDriverSync([("motion_compensation_bypass", False)])
                if not ok:
                    self.get_logger().error("bypass NON retabli : %s" % why)
            self._listen(False)
            with self._lock:
                self._busy = False
        self.get_logger().info(py_trees.display.unicode_tree(root, show_status=True))
        if ctx.abort:
            ctx.note("ABANDON " + ctx.abort[:46])
        elif root.status == Status.SUCCESS:
            ctx.note("FINI : candidat ecrit" if ctx.candidate else "FINI")

        success = root.status == Status.SUCCESS and not ctx.abort
        if gh.is_cancel_requested:
            gh.canceled()
        elif success:
            gh.succeed()
        else:
            gh.abort()
        if success and not goal.imu_check_only:
            self._last = {"variant": goal.variant or SCRATCH_VARIANT, "table": ctx.table}
        res = Calibrate.Result()
        res.success = success
        res.candidate_path = ctx.candidate
        res.summary_json = ctx.report or cm.report_json(
            abandon=ctx.abort, imu=ctx.imu, paliers=ctx.steps, fautes=ctx.faults)
        return res

    def writeCandidate(self, ctx):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        variant = ctx.goal.variant or SCRATCH_VARIANT
        out = self.p["output_dir"] or "/tmp"
        os.makedirs(out, exist_ok=True)
        base = os.path.join(out, "calib_candidat_%s_%s" % (variant, stamp))
        report = cm.report_json(
            robot=self.robot, variante=variant, date=stamp, batterie_v=self.lastBattery,
            ecartement_m=self.wheelSep, imu=ctx.imu, paliers=ctx.steps, asymetrie=ctx.pairs,
            avertissements=ctx.warns, v_bins_mps=self.p["v_bins_mps"],
            w_bins_rps=self.p["w_bins_rps"], yaw_slip=ctx.table, lignes_mesurees=ctx.measured,
            parametres=self.p)
        with open(base + ".json", "w", encoding="utf-8") as fh:
            fh.write(report)
        with open(base + ".yaml", "w", encoding="utf-8") as fh:
            fh.write(cm.candidate_yaml(variant, self.robot, ctx.table, self.p["v_bins_mps"],
                                       self.p["w_bins_rps"], ctx.measured, base + ".json",
                                       self.lastBattery or 0.0, self.wheelSep))
        ctx.candidate, ctx.report = base + ".yaml", report

    # ------------------------------------------------------------ essai en RAM
    def _scratch(self, values):
        items = [("%s.%s.%s" % (PREFIX, SCRATCH_VARIANT, k), values[k]) for k in VARIANT_KEYS]
        return items + [("hardware_variant", SCRATCH_VARIANT)]

    def _onApply(self, _req, resp):
        if self._last is None:
            resp.success, resp.message = False, "aucun candidat en memoire (calibrer d'abord)"
            return resp
        values = dict(self.fileValues)
        values.update(description="candidat %s en RAM" % self._last["variant"],
                      v_bins_mps=self.p["v_bins_mps"], w_bins_rps=self.p["w_bins_rps"],
                      yaw_slip=self._last["table"])
        ok, why = self._setDriverSync(self._scratch(values))
        self._ramActive = self._ramActive or ok
        resp.success = ok
        resp.message = ("candidat %s applique EN RAM (variante %s) : le fichier canonique "
                        "n'est pas modifie" % (self._last["variant"], SCRATCH_VARIANT)
                        if ok else why)
        return resp

    def _onRevert(self, _req, resp):
        ok, why = self._setDriverSync([("hardware_variant", self.fileVariant)])
        if ok:
            self._ramActive = False
        resp.success = ok
        resp.message = ("retour a la variante du fichier : %s" % self.fileVariant) if ok else why
        return resp

    def _onSetPi(self, req, resp):
        if self._ramActive:
            items = []
        else:
            # Pas encore en RAM : on part de la variante du FICHIER, PI en plus.
            items = self._scratch(dict(self.fileValues))
        kp = self.p["pi_test_kp"] if req.data else 0.0
        ki = self.p["pi_test_ki"] if req.data else 0.0
        items += [("%s.%s.yaw_rate_kp" % (PREFIX, SCRATCH_VARIANT), kp),
                  ("%s.%s.yaw_rate_ki" % (PREFIX, SCRATCH_VARIANT), ki)]
        ok, why = self._setDriverSync(items)
        self._ramActive = self._ramActive or ok
        resp.success = ok
        resp.message = ("PI de lacet %s EN RAM (kp %.2f ki %.2f)" % (
            "ACTIF" if req.data else "coupe", kp, ki)) if ok else why
        return resp


def main(argv=None):
    rclpy.init(args=argv)
    node = CalibrationNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:                        # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
