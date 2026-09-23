#!/usr/bin/env python3
"""Lot 4.3 -- critere chiffre « suit le PID », roues SURELEVEES.

Applique un ECHELON de consigne (0 -> linear.x) et mesure, sur /req_states et
/joint_states (RPM vus par la boucle embarquee) :
  - t_conv : delai entre l'apparition de la consigne et la PREMIERE mesure qui
    entre dans la bande +-tol % ET N'EN RESSORT PLUS (une entree suivie d'un
    depassement n'est pas une convergence) ;
  - regime : moyenne, ecart-type et erreur relative sur la fin de l'essai ;
  - oscillation : taux de CHANGEMENT DE SIGNE de l'erreur en regime (~0 =
    convergence franche, >=0.5 = pompage), et amplitude crete-a-crete.

On DEDOUBLONNE les echantillons repetes : le driver publie a ~30 Hz mais la carte
n'emet qu'a 10 Hz, donc trois lignes identiques = UNE mesure. Les compter trois
fois fausserait le taux d'oscillation comme l'ecart-type.

Consigne nulle publiee avant et apres l'echelon : l'integrale du PID embarque
n'est remise a zero que consigne ET erreur nulles, donc chaque essai doit partir
propre. STOP garanti dans le finally.
"""
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

LIN = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
TOL = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0    # bande en %
PRE = 1.0                                                  # s de consigne nulle


class Bench(Node):
    def __init__(self):
        super().__init__("pid_bench_43")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.meas = None
        self.rows = []            # (t, req[4], mes[4]) dedoublonnes
        self.create_subscription(JointState, "/joint_states", self._m, 10)
        self.create_subscription(JointState, "/req_states", self._r, 10)

    def _m(self, msg):
        self.meas = list(msg.velocity)

    def _r(self, msg):
        if self.meas is None:
            return
        req, mes = list(msg.velocity), list(self.meas)
        if self.rows and self.rows[-1][1] == req and self.rows[-1][2] == mes:
            return
        self.rows.append((time.time(), req, mes))


def analyse(rows, tol, idx, label):
    """Analyse UNE voie encodeur (idx 0 = gauche/M1, 1 = droite/M2)."""
    # instant de l'echelon = premiere consigne non nulle
    t_step = next((t for t, r, _ in rows if abs(r[idx]) > 1e-6), None)
    if t_step is None:
        print("  %s : aucun echelon vu." % label)
        return
    seg = [(t - t_step, r[idx], m[idx]) for t, r, m in rows if t >= t_step]
    # on s'arrete au retour a consigne nulle (phase de freinage exclue)
    end = next((i for i, (_, r, _) in enumerate(seg) if abs(r) < 1e-6), len(seg))
    seg = seg[:end]
    if len(seg) < 4:
        print("  %s : trop peu d'echantillons (%d)." % (label, len(seg)))
        return
    req = seg[-1][1]
    band = abs(req) * tol / 100.0

    # convergence : dernier instant ou l'on est HORS bande, + 1 echantillon
    t_conv = None
    for i in range(len(seg) - 1, -1, -1):
        if abs(seg[i][2] - req) > band:
            t_conv = seg[i + 1][0] if i + 1 < len(seg) else None
            break
    else:
        t_conv = seg[0][0]

    # regime = derniere moitie de l'essai
    tail = seg[len(seg) // 2:]
    vals = [m for _, _, m in tail]
    n = len(vals)
    mean = sum(vals) / n
    var = sum((v - mean) ** 2 for v in vals) / n
    err = [m - req for _, _, m in tail]
    flips = sum(1 for a, b in zip(err, err[1:]) if a * b < 0)
    print("  %s : consigne %.2f RPM | t_conv(+-%.0f%%) = %s | regime %.2f +-%.2f "
          "(%.1f %%) | crete-a-crete %.2f | oscillation %.2f"
          % (label, req, tol,
             ("%.2f s" % t_conv) if t_conv is not None else "JAMAIS",
             mean, var ** 0.5, (mean - req) / req * 100.0,
             max(vals) - min(vals), flips / max(1, n - 1)))


def main():
    rclpy.init()
    n = Bench()
    tw, zero = Twist(), Twist()
    tw.linear.x = LIN
    t0 = time.time()
    try:
        while time.time() - t0 < PRE:
            n.pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.05)
        while time.time() - t0 < PRE + DUR:
            n.pub.publish(tw)
            rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        for _ in range(20):
            n.pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.05)
    print("=== linear.x = %.2f m/s, %d echantillons carte ===" % (LIN, len(n.rows)))
    analyse(n.rows, TOL, 0, "M1 gauche")
    analyse(n.rows, TOL, 1, "M2 droite")
    rclpy.shutdown()


main()
