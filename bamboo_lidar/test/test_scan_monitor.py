"""ScanStats, sans ROS."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_lidar.scan_monitor import ScanStats  # noqa: E402


def test_ok_rate_and_ranges():
    s = ScanStats()
    for k in range(30):
        s.add(k * 0.1, [1.0] * 450 + [math.inf] * 50, 0.05, 12.0)
    lvl, msg = s.verdict(2.95, 3.0, 7.0, 0.3)
    assert lvl == 0 and "450/500" in msg
    assert 9.5 < s.rate(2.9) < 10.5


def test_silent_slow_and_masked():
    s = ScanStats()
    assert s.verdict(0.0, 3.0, 7.0, 0.3)[0] == 2
    for k in range(10):
        s.add(k * 0.5, [1.0] * 500, 0.05, 12.0)            # 2 Hz
    assert s.verdict(4.6, 3.0, 7.0, 0.3)[0] == 1
    assert s.verdict(9.0, 3.0, 7.0, 0.3)[0] == 2           # muet depuis 4,5 s
    m = ScanStats()
    for k in range(30):
        m.add(k * 0.1, [0.0] * 400 + [1.0] * 100, 0.05, 12.0)
    assert "points valides" in m.verdict(2.95, 3.0, 7.0, 0.3)[1]
