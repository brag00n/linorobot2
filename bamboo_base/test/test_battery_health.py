"""Estimateur de sante batterie, sur des series SYNTHETIQUES (sans ROS)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_base.battery_health import BatteryHealth, soc_from_rest  # noqa: E402


def test_soc_curve():
    assert soc_from_rest(12.6, 3) == 100.0
    assert soc_from_rest(9.0, 3) == 0.0
    assert 20 < soc_from_rest(11.1, 3) < 30          # 3,70 V/element


def _drive(h, r_ohm, v0=12.0, t0=0.0, n=600, req=60.0, track=1.0):
    """Alterne repos (0,1 A) et charge (2,5 A) ; V = V0 - R*I."""
    for k in range(n):
        t = t0 + 0.25 * k
        loaded = (k // 20) % 2 == 1
        i = 2.5 if loaded else 0.1
        r = [req, -req] if loaded else [0.0, 0.0]
        m = [x * track for x in r]
        h.observe(t, v0 - r_ohm * i, i, r, m)


def test_resistance_rest_and_sag():
    h = BatteryHealth()
    _drive(h, 0.18)
    assert math.isclose(h.resistance, 0.18, rel_tol=0.02)
    assert math.isclose(h.rest_voltage, 12.0 - 0.018, abs_tol=0.01)
    assert math.isclose(h.sag, 0.18 * 2.4, abs_tol=0.02)
    assert h.verdict(20, 0.0, 0.0, 0.25) == []          # seuils R/chute desactives
    assert h.verdict(20, 0.15, 0.0, 0.25)               # R au-dessus du seuil


def test_traction_deficit():
    h = BatteryHealth()
    _drive(h, 0.1, track=0.5)                            # roues a 50 % de la consigne
    frac, vbad, total = h.deficit
    assert frac > 0.9 and total > 20
    assert any("sous consigne" in w for w in h.verdict(20, 0.0, 0.0, 0.25))


def test_no_current_falls_back_on_commands():
    h = BatteryHealth()
    for k in range(40):
        h.observe(0.25 * k, 11.8, None, [0.0, 0.0], [0.0, 0.0])
    assert h.resistance is None and h.rest_voltage == 11.8
