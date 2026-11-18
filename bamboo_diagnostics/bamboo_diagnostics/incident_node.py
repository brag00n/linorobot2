"""incident_node : de /diagnostics_agg a /bamboo/incidents (interpretation) et a une tache standard de /diagnostics (synthese). LECTURE SEULE.

Trois topics, trois roles, jamais de doublon :
  /diagnostics        les FAITS (standard ROS). Ce noeud n'y ajoute qu'UNE tache, `incident_node: synthese`, pour que Foxglove ou n'importe quel outil ROS voie aussi l'interpretation.
  /diagnostics_agg    les memes faits, ranges en arbre. C'est l'ENTREE de ce noeud.
  /bamboo/incidents   l'INTERPRETATION : racines, consequences, impact, pistes de resolution (JSON dans un std_msgs/String, latche, publie seulement quand le contenu change).

Aucun service appele, aucune ecriture hors de ces deux sorties. Le calcul vit dans incident_model.py (Python pur, teste hors ROS).
"""
import json
import os
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from bamboo_diagnostics import incident_model as im
from bamboo_diagnostics import parts_model as pm


class IncidentNode(Node):
    def __init__(self):
        super().__init__("incident_node")
        p = self.declare_parameter
        parts_path = str(p("parts_file", "").value)
        if not parts_path:
            raise RuntimeError("parametre parts_file requis (chemin de parts.yaml du robot)")
        self._doc = pm.load(parts_path)
        errs = pm.validate(self._doc)
        if errs:                                                           # une nomenclature fausse ne doit pas produire d'incidents faux
            raise RuntimeError("parts.yaml invalide :\n  " + "\n  ".join(errs))
        self._graph = pm.Graph(self._doc)
        self._idx = im.index_diag(self._doc)
        self._tracker = im.IncidentTracker(
            self._graph, confirmations=int(p("confirmations", 2).value), garde_resolu_s=float(p("garde_resolu_s", 30.0).value))
        self._odDir = str(p("ondemand_dir", "/run/bamboo-ondemand").value)
        self._last = ""                                                    # derniere charge publiee (comparaison)
        self._lastPub = 0.0
        self._heartbeat = float(p("republier_toutes_s", 30.0).value)       # republication meme sans changement : un lecteur tardif ne reste pas sans etat

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pubInc = self.create_publisher(String, "/bamboo/incidents", latched)
        self._pubDiag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(DiagnosticArray, "/diagnostics_agg", self._onAgg, 10)
        self._synth = (im.OK, "en attente du premier arbre", [])
        self.create_timer(1.0, self._publishSynthese)
        self.get_logger().info("incident_node pret : %d parts, %d taches rattachees, confirmation sur %d releves"
                               % (len(self._doc["parts"]), len(self._idx), self._tracker.confirmations))

    def _hostTracking(self):
        """(etat, anciennete s) du groupe `tracking` cote hote, comme le fait le teleop. (None, None) si illisible."""
        try:
            with open(os.path.join(self._odDir, "tracking.state")) as fh:
                st = fh.read().split()
            return (st[0], max(0.0, time.time() - float(st[-1]))) if len(st) >= 2 else (st[0] if st else None, None)
        except (OSError, ValueError, IndexError):
            return None, None

    def _onAgg(self, msg):
        f = im.feuilles(msg.status)
        hs, age = self._hostTracking()
        bad, par_part, orph, ign = im.parts_en_defaut(f, self._idx, hs, age)
        actifs, resolus = self._tracker.update(bad, par_part, f)
        body = im.payload(actifs, resolus, orph, ign, len(f))
        # Comparaison SANS les champs qui changent a chaque releve (anciennete), sinon on republierait toutes les 3 s
        stable_cmp = json.dumps(self._sans_horloge(body), sort_keys=True, ensure_ascii=False)
        now = time.monotonic()
        if stable_cmp != self._last or now - self._lastPub > self._heartbeat:
            self._last, self._lastPub = stable_cmp, now
            m = String()
            m.data = json.dumps(body, ensure_ascii=False, separators=(",", ":"))
            self._pubInc.publish(m)
            if actifs:
                self.get_logger().warn("incidents : " + "; ".join("%s <- %s" % (a["racine"], ", ".join(a["taches"])) for a in actifs))
        self._synth = im.synthese(actifs, orph)

    @staticmethod
    def _sans_horloge(body):
        b = json.loads(json.dumps(body, default=str))
        for k in ("incidents", "resolus_recents"):
            for x in b.get(k, []):
                x.pop("depuis_s", None)
                x.pop("resolu_il_y_a_s", None)
        return b

    def _publishSynthese(self):
        lvl, msg, vals = self._synth
        s = DiagnosticStatus(name=im.SELF_TASK, hardware_id="bamboo_diagnostics", level=bytes([lvl]), message=msg)
        s.values = [KeyValue(key=k, value=v) for k, v in vals]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [s]
        self._pubDiag.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = IncidentNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):      # SIGINT / SIGTERM (docker stop, timeout) : arret normal, pas une erreur
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
