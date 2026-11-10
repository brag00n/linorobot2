"""Noeud `battery_energy` : statistiques d'energie en temps reel (COLLECTE seule, aucune decision).

Demande utilisateur du 2026-11-10 : calculer l'autonomie en temps reel pour, plus tard, envoyer le
robot se recharger en gardant de quoi rentrer a sa base. Ce noeud ne pilote RIEN : il lit
/battery, /odom/unfiltered (vitesses mesurees aux roues) et /navigation/state, puis

  * publie /bamboo/energy (std_msgs/String, JSON, 1 Hz) : courant et puissance par activite,
    energie depuis le debranchement du chargeur, resistance interne en roulage, autonomie
    PROVISOIRE (`fiable: false` tant qu'une decharge reelle ne l'a pas confirmee) ;
  * ecrit un journal CSV a 1 Hz (une ligne = tension, courant, vitesse, activite, but de
    navigation en cours, charge CPU...) pour ajuster les criteres hors ligne ;
  * ecrit un resume par SEGMENT sur batterie (du debranchement au rebranchement du chargeur).

Les fichiers ont une taille BORNEE (voir energy_stats.RotatingLog) : rotation, compression,
purge des plus anciens, suspension si le disque est presque plein.

Topic en String plutot qu'en message dedie : evite de recompiler bamboo_interfaces (long sur le
RPi) tant que le contenu evolue ; il sera fige en message le jour ou la decision en depend.
"""
import json
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from bamboo_base.energy_stats import EnergyStats, JsonlTail, RotatingLog

CSV_COLS = ["epoch", "charge", "v_V", "i_A", "soc_pct", "vx_ms", "wz_rads", "activite", "p_W",
            "ah_segment", "dist_segment_m", "i_ema_A", "autonomie_min", "r_ohm", "charge_cpu",
            "mode_nav", "but", "but_statut", "dist_restante_m", "localisation"]


class BatteryEnergy(Node):
    def __init__(self):
        super().__init__("battery_energy")
        d = self.declare_parameter
        d("batt_capacity_ah", 0.88)           # capacite mesuree du pack (calibration du 2026-10-22)
        d("log_dir", "/root/linorobot2_ws/src/linorobot2/data/energy")
        d("log_max_file_mb", 2.0)             # rotation d'un fichier CSV
        d("log_max_total_mb", 30.0)           # plafond de TOUS les CSV (anciens purges)
        d("log_min_free_mb", 1024.0)          # sous ce libre, plus aucune ecriture
        d("log_enabled", True)
        d("publish_hz", 1.0)
        d("battery_topic", "/battery")
        d("odom_topic", "/odom/unfiltered")
        d("ema_s", 60.0)                      # lissage du courant moyen
        d("r_window_s", 120.0)                # fenetre de la regression de resistance
        d("reliable_after_s", 300.0)          # duree sur batterie avant d'annoncer `fiable`
        g = lambda n: self.get_parameter(n).value
        self.stats = EnergyStats(capacity_ah=float(g("batt_capacity_ah")), ema_s=float(g("ema_s")),
                                 r_window_s=float(g("r_window_s")),
                                 reliable_after_s=float(g("reliable_after_s")))
        self.log = None
        self.segments = None
        if bool(g("log_enabled")):
            try:
                out = str(g("log_dir"))
                self.log = RotatingLog(out, "energie", CSV_COLS,
                                       max_bytes=int(float(g("log_max_file_mb")) * (1 << 20)),
                                       max_total=int(float(g("log_max_total_mb")) * (1 << 20)),
                                       min_free=int(float(g("log_min_free_mb")) * (1 << 20)))
                self.segments = JsonlTail(os.path.join(out, "segments.jsonl"))
            except OSError as e:
                self.get_logger().warn("journal d'energie desactive (%s : %s)" % (g("log_dir"), e))
        self.vx = self.wz = 0.0
        self._odomT = 0.0
        self._lastUpdate = 0.0
        self._nav = {}
        self.pub = self.create_publisher(String, "/bamboo/energy", 1)
        self.create_subscription(BatteryState, str(g("battery_topic")), self._onBattery, 5)
        self.create_subscription(Odometry, str(g("odom_topic")), self._onOdom, 5)
        try:
            from bamboo_interfaces.msg import NavState
            self.create_subscription(NavState, "/navigation/state", self._onNav, 1)
        except ImportError:
            self.get_logger().warn("bamboo_interfaces absent : le but de navigation n'est pas journalise")
        self.create_timer(1.0 / max(0.1, float(g("publish_hz"))), self._tick)
        self.get_logger().info("battery_energy : capacite %.2f Ah, journal %s" % (
            self.stats.capacity_ah, "desactive" if self.log is None else self.log.dir))

    # ---- entrees ----

    def _onOdom(self, m):
        self.vx = float(m.twist.twist.linear.x)
        self.wz = float(m.twist.twist.angular.z)
        self._odomT = time.monotonic()

    def _onNav(self, m):
        self._nav = {"mode": m.submode, "but": m.goal, "statut": m.goal_status,
                     "dist": float(m.distance_remaining), "loc": m.loc_state}

    def _onBattery(self, m):
        now = time.monotonic()
        if now - self._lastUpdate < 0.2:        # 5 Hz suffisent, /battery arrive a 20 Hz
            return
        v, cur = float(m.voltage), float(m.current)
        if not v > 0.0 or math.isnan(cur):
            return
        self._lastUpdate = now
        pct = float(m.percentage)
        soc = 100.0 * pct if pct == pct and 0.0 <= pct <= 1.0 else None
        # Odometrie muette depuis plus de 2 s : on ne sait plus si le robot roule -> arret.
        stale = now - self._odomT > 2.0
        vx, wz = (0.0, 0.0) if stale else (self.vx, self.wz)
        self.stats.update(now, v, -cur, vx, wz,
                          m.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING, soc)

    # ---- 1 Hz ----

    def _tick(self):
        s = self.stats.snapshot()
        out = String()
        out.data = json.dumps(s, separators=(",", ":"))
        self.pub.publish(out)
        summary = self.stats.take_summary()
        if summary is not None and self.segments is not None:
            summary["epoch"] = round(time.time(), 0)
            self.segments.append(summary)
            self.get_logger().info("segment sur batterie : %s s, %s Ah, %s m" % (
                summary["duree_s"], summary["ah"], summary["dist_m"]))
        if self.log is None or s["v"] is None:
            return
        seg = s["segment"] or {}
        p = None if s["v"] is None or s["i"] is None else round(s["v"] * s["i"], 2)
        try:
            load = round(os.getloadavg()[0], 2)
        except OSError:
            load = None
        n = self._nav
        dist = n.get("dist")
        self.log.write([
            round(time.time(), 1), int(s["charge"]), s["v"], s["i"], s["soc"],
            round(self.vx, 3), round(self.wz, 3), s["activite"], p,
            seg.get("ah"), seg.get("dist_m"), s["i_ema"], s["autonomie_min"], s["r_ohm"], load,
            n.get("mode"), n.get("but"), n.get("statut"),
            None if dist is None or dist != dist else round(dist, 2), n.get("loc")])

    def shutdown(self):
        self.stats.end_segment()
        summary = self.stats.take_summary()
        if summary is not None and self.segments is not None:
            summary["epoch"] = round(time.time(), 0)
            self.segments.append(summary)
        if self.log is not None:
            self.log.close()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryEnergy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
