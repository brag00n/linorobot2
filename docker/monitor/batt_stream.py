#!/usr/bin/env python3
"""Flux /battery sur la sortie standard, une ligne JSON par message.

Lance DANS le conteneur du driver (il a ROS) par historique_batterie.py, qui tourne sur l'hote
et lit ce flux par un tube `docker exec -i`. Rien n'est ecrit ici : ce script ne fait que
traduire un topic ROS en lignes de texte.

Champs : t (s, horloge de l'hote), v (V), i (A CONSOMMES, positif en decharge), pct (% ou null
tant que le chargeur est branche), st (power_supply_status : 0 inconnu, 1 charge, 2 decharge).
"""
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BattStream(Node):

    def __init__(self):
        super().__init__("batt_stream")
        self.create_subscription(BatteryState, "/battery", self._on_msg, 10)

    @staticmethod
    def _on_msg(m):
        pct = None if m.percentage != m.percentage else round(100.0 * m.percentage, 1)
        print(json.dumps({"t": round(time.time(), 3), "v": round(m.voltage, 3),
                          "i": round(-m.current, 3), "pct": pct,
                          "st": int(m.power_supply_status)}), flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(BattStream())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
