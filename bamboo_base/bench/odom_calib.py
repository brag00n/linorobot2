#!/usr/bin/env python3
"""Lot 4.4 -- calibration de l'odometrie AU SOL.

Deux essais independants, chacun isolant UNE constante geometrique :

  mode « ligne » : avance d'une distance visee en ligne droite. Comparer la
                   distance ODOM a la distance REELLE au metre ruban calibre
                   `wheel_diameter_m` -- le diametre est le seul facteur d'echelle
                   entre tours de roue et metres.
  mode « tour »  : tourne sur place d'un tour complet. Comparer l'angle ODOM a
                   l'angle REEL (repere au sol) calibre `wheel_separation_m` --
                   la voie est le seul facteur entre difference de rotation des
                   roues et rotation du chassis.

Les corriger dans le MEME essai serait impossible a demeler : une erreur de
diametre deplace aussi l'angle. D'ou deux passes, dans cet ordre.

Correction a appliquer ensuite dans le fichier canonique :
    wheel_diameter_m   *= reel / odom     (essai ligne)
    wheel_separation_m *= odom / reel     (essai tour ; sens inverse, la voie est
                                           au DENOMINATEUR de la vitesse angulaire)

L'arret est donne par l'ODOM (boucle fermee sur la mesure), pas par un chronometre :
une duree fixe dependrait de l'acceleration. Rampe et STOP garanti dans le finally.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

MODE = sys.argv[1] if len(sys.argv) > 1 else "ligne"        # ligne | tour
CIBLE = float(sys.argv[2]) if len(sys.argv) > 2 else (2.0 if MODE == "ligne" else 2 * math.pi)
VIT = float(sys.argv[3]) if len(sys.argv) > 3 else (0.15 if MODE == "ligne" else 0.8)
GARDE = 25.0                                                # s : coupe-circuit de duree


def yaw_de(q):
    """Yaw d'un quaternion ROS (roulis et tangage negligeables sur un skid-steer)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Calib(Node):
    def __init__(self):
        super().__init__("odom_calib_44")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.p0 = None            # (x, y, yaw) au depart
        self.cur = None
        self.yaw_cum = 0.0        # yaw DEROULE (l'odom renvoie un angle module 2*pi)
        self._yaw_prec = None
        self._yaw0 = None         # cap dans `odom` au debut de l'essai
        self.create_subscription(Odometry, "/odom/unfiltered", self._o, 20)

    def _o(self, msg):
        p = msg.pose.pose
        y = yaw_de(p.orientation)
        if self._yaw_prec is not None:
            d = y - self._yaw_prec
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            self.yaw_cum += d
        self._yaw_prec = y
        self.cur = (p.position.x, p.position.y, self.yaw_cum)
        if self.p0 is None:
            self.p0 = self.cur

    def avance(self):
        """Distance euclidienne parcourue depuis le depart, en m."""
        if self.p0 is None or self.cur is None:
            return 0.0
        return math.hypot(self.cur[0] - self.p0[0], self.cur[1] - self.p0[1])

    def ecart(self):
        """Deplacement dans le repere du DEPART : (avant, lateral) en m.

        `y - y0` brut ne veut rien dire : x et y sont dans le repere `odom`, dont
        l'origine et le cap datent du demarrage du driver, pas de cet essai. Mesure
        du 2026-09-23 : 0,78 m de "derive laterale" pour 3,85 deg de rotation reelle
        sur 2,1 m -- geometriquement impossible, c'etait juste le cap initial du robot
        dans `odom`. On projette donc le deplacement sur le cap de depart.
        """
        if self.p0 is None or self.cur is None or self._yaw0 is None:
            return 0.0, 0.0
        dx, dy = self.cur[0] - self.p0[0], self.cur[1] - self.p0[1]
        c, s = math.cos(-self._yaw0), math.sin(-self._yaw0)
        return dx * c - dy * s, dx * s + dy * c

    def rotation(self):
        """Angle deroule parcouru depuis le depart, en rad."""
        if self.p0 is None or self.cur is None:
            return 0.0
        return self.cur[2] - self.p0[2]


def main():
    rclpy.init()
    n = Calib()
    tw, zero = Twist(), Twist()
    if MODE == "ligne":
        tw.linear.x = VIT
    else:
        tw.angular.z = VIT
    mesure = n.avance if MODE == "ligne" else (lambda: abs(n.rotation()))

    # on attend une premiere odom : sans origine, la boucle d'arret est aveugle
    t0 = time.time()
    while n.p0 is None and time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.p0 is None:
        print("PAS D'ODOM sur /odom/unfiltered -- essai annule.")
        rclpy.shutdown()
        return

    t0 = time.time()
    try:
        while time.time() - t0 < 1.0:          # integrale PID propre
            n.pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.05)
        n.p0 = n.cur                            # origine APRES la phase nulle
        n._yaw0 = n._yaw_prec                   # cap BRUT dans `odom` (pas le cumul)
        while mesure() < CIBLE:
            if time.time() - t0 > GARDE:
                print("GARDE DE DUREE ATTEINTE (%.0f s) -- arret." % GARDE)
                break
            n.pub.publish(tw)
            rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        for _ in range(25):
            n.pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.05)

    # l'inertie continue apres la coupure : on relit l'odom une fois immobile
    time.sleep(0.5)
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)

    if MODE == "ligne":
        print("=== ligne droite, %.2f m/s ===" % VIT)
        print("  distance ODOM      = %.4f m   (cible %.2f m)" % (n.avance(), CIBLE))
        av, lat = n.ecart()
        print("  avance / lateral   = %.4f m / %.4f m  (repere du depart)" % (av, lat))
        print("  rotation parasite  = %.2f deg" % math.degrees(n.rotation()))
        print("  -> mesurer la distance REELLE, puis wheel_diameter_m *= reel / ODOM")
    else:
        print("=== rotation sur place, %.2f rad/s ===" % VIT)
        print("  angle ODOM         = %.4f rad = %.2f deg   (cible %.2f deg)"
              % (n.rotation(), math.degrees(n.rotation()), math.degrees(CIBLE)))
        print("  deplacement parasite = %.4f m" % n.avance())
        print("  -> mesurer l'angle REEL, puis wheel_separation_m *= ODOM / reel")
    rclpy.shutdown()


main()
