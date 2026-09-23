#!/usr/bin/env python3
"""Diagnostic : l'odom integree en VITESSE contre les COMPTAGES bruts.

L'odom du driver integre `tics/s * dt` (esp32_mavlink_driver.py). Les comptages
cumulatifs sont pourtant disponibles, et leur DIFFERENCE est exacte : elle ne
depend ni de la cadence du lien, ni de l'horloge de la carte, ni du dt du timer.

Les deux chemins partagent le MEME diametre et le MEME cpr. Leur rapport isole
donc l'erreur du chemin vitesse, SANS metre ruban et sans connaitre la geometrie :

    ratio = distance_odom / distance_comptages

  ratio ~ 1      -> le chemin vitesse est sain ; un ecart au ruban vient alors de
                    la geometrie (diametre) ou du glissement des roues.
  ratio ~ 1.09   -> c'est le chemin vitesse qui gonfle, la geometrie est innocente
                    et la corriger graverait l'erreur au mauvais endroit.

`position` de /joint_states porte deja les comptages convertis en radians par le
driver (c / cpr * 2*pi), donc distance_roue = dtheta * D / 2. On lit les DEUX
voies encodeur et on en prend la moyenne, comme le fait l'odom.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

VIT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
DIAM = float(sys.argv[3]) if len(sys.argv) > 3 else 0.08


class Cmp(Node):
    def __init__(self):
        super().__init__("odom_vs_counts")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.xy = None
        self.xy0 = None
        self.pos = None           # position[4] en radians
        self.pos0 = None
        self.create_subscription(Odometry, "/odom/unfiltered", self._o, 20)
        self.create_subscription(JointState, "/joint_states", self._j, 20)

    def _o(self, m):
        p = m.pose.pose.position
        self.xy = (p.x, p.y)

    def _j(self, m):
        if len(m.position) >= 2:
            self.pos = list(m.position)

    def reset(self):
        self.xy0, self.pos0 = self.xy, self.pos

    def d_odom(self):
        if self.xy0 is None or self.xy is None:
            return 0.0
        return math.hypot(self.xy[0] - self.xy0[0], self.xy[1] - self.xy0[1])

    def d_counts(self):
        """Moyenne des deux voies encodeur : dtheta * rayon."""
        if self.pos0 is None or self.pos is None:
            return 0.0
        g = abs(self.pos[0] - self.pos0[0])       # M1 gauche
        d = abs(self.pos[1] - self.pos0[1])       # M2 droite
        return (g + d) / 2.0 * DIAM / 2.0


def main():
    rclpy.init()
    n = Cmp()
    tw, zero = Twist(), Twist()
    tw.linear.x = VIT
    t0 = time.time()
    while (n.xy is None or n.pos is None) and time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.xy is None or n.pos is None:
        print("odom ou /joint_states absent -- essai annule.")
        rclpy.shutdown()
        return
    t0 = time.time()
    try:
        while time.time() - t0 < 1.0:
            n.pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.05)
        n.reset()
        while time.time() - t0 < 1.0 + DUR:
            n.pub.publish(tw)
            rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        for _ in range(25):
            n.pub.publish(zero)
            rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.6)
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)

    do, dc = n.d_odom(), n.d_counts()
    print("=== %.2f m/s pendant %.1f s, diametre %.4f m ===" % (VIT, DUR, DIAM))
    print("  distance ODOM (vitesse integree) = %.4f m" % do)
    print("  distance COMPTAGES (exacte)      = %.4f m" % dc)
    print("  ratio odom/comptages             = %.4f  (ecart %+.1f %%)"
          % (do / dc if dc > 1e-9 else float("nan"),
             (do / dc - 1.0) * 100.0 if dc > 1e-9 else float("nan")))
    rclpy.shutdown()


main()
