#!/usr/bin/env python3
"""Temoin de reglage PID : publie un cmd_vel borne et echantillonne, sur le MEME
instant, la vitesse DEMANDEE (/req_states) et la vitesse MESUREE (/joint_states).
Roues surelevees. STOP garanti dans le finally."""
import sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

LIN = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0


class Witness(Node):
    def __init__(self):
        super().__init__("pid_witness")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.meas = None
        self.req = None
        self.rows = []
        self.create_subscription(JointState, "/joint_states", self._m, 10)
        self.create_subscription(JointState, "/req_states", self._r, 10)

    def _m(self, msg):
        self.meas = list(msg.velocity)

    def _r(self, msg):
        self.req = list(msg.velocity)
        # On echantillonne sur la consigne : les deux messages sortent du meme tick
        # du driver, donc la mesure associee est celle du meme cycle PID.
        if self.meas is not None:
            self.rows.append((time.time(), list(self.req), list(self.meas)))


def main():
    rclpy.init()
    n = Witness()
    t0 = time.time()
    tw = Twist()
    tw.linear.x = LIN
    try:
        while time.time() - t0 < DUR:
            n.pub.publish(tw)
            rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        z = Twist()
        for _ in range(5):
            n.pub.publish(z)
            rclpy.spin_once(n, timeout_sec=0.02)
        time.sleep(0.3)
        for _ in range(10):
            rclpy.spin_once(n, timeout_sec=0.05)
    print("t_rel  req(M1..M4)                 mes(M1..M4)")
    for t, r, m in n.rows:
        print("%5.2f  [%s]  [%s]" % (t - t0,
              " ".join("%7.2f" % v for v in r),
              " ".join("%7.2f" % v for v in m)))
    rclpy.shutdown()


main()
