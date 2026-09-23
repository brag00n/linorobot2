#!/usr/bin/env python3
"""Lot 4.2 -- temps de FREINAGE, robot AU SOL.

Deux chemins d'arret existent et ne coutent pas le meme temps ; on les mesure
separement au lieu de les confondre dans un seul chiffre :

  mode « zero »  : on publie explicitement Twist(0,0). C'est le seul frein REEL
                   (link.stop() est un no-op sur l'ESP32, son case BAMBOO_MOTOR_PWM
                   etant vide) -> borne basse du freinage.
  mode « mute »  : on ARRETE de publier. Le freinage attend alors le watchdog du
                   driver (0,5 s) puis, a defaut, le fail-safe carte (500 ms). C'est
                   le cas d'une perte de manette ou d'un nœud qui meurt : le critere
                   du plan (< 0,5 s) porte sur CE chemin.

Le t=0 est le dernier Twist non nul publie. On mesure jusqu'a ce que les DEUX voies
encodeur restent sous un seuil de RPM. Mesure prise sur /joint_states, dedoublonnee
(driver ~30 Hz, carte 10 Hz). STOP garanti dans le finally, quel que soit le mode.
"""
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

MODE = sys.argv[1] if len(sys.argv) > 1 else "zero"        # zero | mute
LIN = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0     # duree de mise en vitesse
SEUIL = 1.0                                                # RPM considere comme nul
WATCH = 2.5                                                # s d'observation apres t=0
CIRC_M = 0.2513                                            # circonference roue (fichier canonique)


class Brake(Node):
    def __init__(self):
        super().__init__("brake_bench_42")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.rows = []            # (t, [rpm M1..M4]) dedoublonnes
        self.create_subscription(JointState, "/joint_states", self._m, 10)

    def _m(self, msg):
        v = list(msg.velocity)
        if self.rows and self.rows[-1][1] == v:
            return
        self.rows.append((time.time(), v))


def main():
    rclpy.init()
    n = Brake()
    tw, zero = Twist(), Twist()
    tw.linear.x = LIN
    t0 = time.time()
    t_cut = None
    try:
        while time.time() - t0 < 1.0:      # consigne nulle : integrale PID propre
            n.pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.05)
        while time.time() - t0 < 1.0 + DUR:
            n.pub.publish(tw)
            rclpy.spin_once(n, timeout_sec=0.05)
        t_cut = time.time()                # dernier Twist non nul = t=0
        t1 = time.time()
        while time.time() - t1 < WATCH:
            if MODE == "zero":
                n.pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        for _ in range(20):                # STOP inconditionnel
            n.pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.05)

    seg = [(t - t_cut, v) for t, v in n.rows if t >= t_cut]
    if not seg:
        print("=== aucun echantillon apres la coupure ===")
        rclpy.shutdown()
        return
    v0 = (abs(seg[0][1][0]) + abs(seg[0][1][1])) / 2.0    # RPM moyen a la coupure

    def premier_sous(seuil):
        """Premier instant ou les deux voies passent sous `seuil` ET N'EN RESSORTENT PLUS."""
        for i, (dt, _) in enumerate(seg):
            if all(abs(v[0]) < seuil and abs(v[1]) < seuil for _, v in seg[i:]):
                return dt
        return None

    t_stop = premier_sous(SEUIL)
    t_90 = premier_sous(0.10 * v0)                        # 90 % de la vitesse perdue

    # DISTANCE parcourue depuis la coupure : integrale trapezoidale du RPM moyen.
    # C'est la grandeur qui compte pour la securite -- une longue queue a 3 RPM
    # coute des CENTIMETRES, la lire en secondes seules donne une fausse alerte.
    dist = 0.0
    for (t_a, va), (t_b, vb) in zip(seg, seg[1:]):
        ra = (abs(va[0]) + abs(va[1])) / 2.0
        rb = (abs(vb[0]) + abs(vb[1])) / 2.0
        dist += (ra + rb) / 2.0 / 60.0 * (t_b - t_a)      # tours
    dist *= CIRC_M                                        # metres

    print("=== freinage mode %s, linear.x = %.2f m/s, %d echantillons ===" % (MODE, LIN, len(seg)))
    print("  RPM a la coupure : M1 %.2f | M2 %.2f" % (seg[0][1][0], seg[0][1][1]))
    print("  t_-90%% (sous %.1f RPM)   = %s"
          % (0.10 * v0, ("%.2f s" % t_90) if t_90 is not None else "JAMAIS"))
    print("  t_arret (< %.1f RPM)     = %s"
          % (SEUIL, ("%.2f s" % t_stop) if t_stop is not None else "JAMAIS"))
    print("  distance apres coupure   = %.3f m (%.1f cm)" % (dist, dist * 100.0))
    print("  trace : " + " ".join("%.2fs:%.0f/%.0f" % (dt, v[0], v[1]) for dt, v in seg[:14]))
    rclpy.shutdown()


main()
