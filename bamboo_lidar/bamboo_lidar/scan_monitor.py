#!/usr/bin/env python3
"""scan_monitor -- diagnostic du lidar : le scan arrive-t-il, a quelle cadence, et voit-il ?

Le driver du lidar (ldlidar_stl_ros2) ne publie AUCUN diagnostic : s'il se tait, rien ne le
dit, et la navigation tomberait sans raison visible. Ce noeud ecoute le sujet du scan (nom
REEL, donne par le wiring du robot) et publie la tache `lidar` sur /diagnostics :

  ERROR  aucun scan depuis `stale_s` (driver arrete, lidar debranche, mauvais port)
  WARN   cadence sous `warn_hz` (le D500 tourne a ~10 Hz), ou trop peu de points valides
         (capteur masque, mauvais montage, recadrage trop large)
  OK     cadence, points valides, portee observee

Le calcul est en Python pur (ScanStats), teste sans ROS.
"""
import collections
import math
import time


class ScanStats:
    """Statistiques glissantes d'un flux de LaserScan (temps, liste de distances)."""

    def __init__(self, window_s=5.0):
        self.window_s = window_s
        self._t = collections.deque()
        self.points = 0
        self.valid = 0
        self.rmin = self.rmax = None
        self.last_t = None

    def add(self, t, ranges, range_min, range_max):
        self._t.append(t)
        while self._t and t - self._t[0] > self.window_s:
            self._t.popleft()
        ok = [r for r in ranges if math.isfinite(r) and range_min <= r <= range_max]
        self.points, self.valid = len(ranges), len(ok)
        self.rmin = min(ok) if ok else None
        self.rmax = max(ok) if ok else None
        self.last_t = t

    def rate(self, now):
        ts = [x for x in self._t if now - x <= self.window_s]
        if len(ts) < 2:
            return 0.0
        return (len(ts) - 1) / max(1e-3, ts[-1] - ts[0])

    def verdict(self, now, stale_s, warn_hz, min_valid_ratio):
        """(niveau 0/1/2, message)."""
        if self.last_t is None:
            return 2, "aucun scan recu (driver arrete ? lidar debranche ?)"
        age = now - self.last_t
        if age > stale_s:
            return 2, "plus de scan depuis %.1f s" % age
        hz = self.rate(now)
        ratio = self.valid / self.points if self.points else 0.0
        if hz < warn_hz:
            return 1, "cadence %.1f Hz (< %.0f)" % (hz, warn_hz)
        if ratio < min_valid_ratio:
            return 1, "%.0f %% de points valides seulement (capteur masque ?)" % (100 * ratio)
        return 0, "%.1f Hz, %d/%d points valides, %.2f-%.1f m" % (
            hz, self.valid, self.points, self.rmin or 0.0, self.rmax or 0.0)


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from diagnostic_msgs.msg import DiagnosticStatus
    from diagnostic_updater import Updater
    from sensor_msgs.msg import LaserScan

    class ScanMonitor(Node):
        def __init__(self):
            super().__init__("scan_monitor")
            d = self.declare_parameter
            topic = str(d("scan_topic", "/scan").value)
            self.stale_s = float(d("stale_s", 3.0).value)
            self.warn_hz = float(d("warn_hz", 7.0).value)
            self.min_valid = float(d("min_valid_ratio", 0.3).value)
            self.stats = ScanStats(float(d("window_s", 5.0).value))
            self.topic = topic
            # sensor_data : compatible avec un driver reliable comme best-effort.
            self.create_subscription(LaserScan, topic, self._onScan, qos_profile_sensor_data)
            self.updater = Updater(self)
            self.updater.setHardwareID(str(d("hardware_id", "driver.lidarD500").value))
            self.updater.add("lidar", self._diag)
            self.get_logger().info("surveillance du scan sur %s" % topic)

        def _onScan(self, m):
            self.stats.add(time.monotonic(), list(m.ranges), m.range_min, m.range_max)

        def _diag(self, stat):
            now = time.monotonic()
            s = self.stats
            stat.add("sujet", self.topic)
            stat.add("cadence_hz", "%.1f" % s.rate(now))
            stat.add("points", str(s.points))
            stat.add("points_valides", str(s.valid))
            stat.add("portee_m", "--" if s.rmin is None else "%.2f-%.2f" % (s.rmin, s.rmax))
            lvl, msg = s.verdict(now, self.stale_s, self.warn_hz, self.min_valid)
            stat.summary((DiagnosticStatus.OK, DiagnosticStatus.WARN,
                          DiagnosticStatus.ERROR)[lvl], msg)
            return stat

    rclpy.init(args=argv)
    node = ScanMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
