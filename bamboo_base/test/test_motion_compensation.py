"""Tests du coeur de la compensation de rotation (Python pur, sans ROS).

Le premier test est le contrat du lot : TOUT A ZERO = IDENTITE EXACTE (egalite, pas
approximation), consigne comme odometrie.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_base.motion_compensation import (  # noqa: E402
    VARIANT_KEYS, CompensationCore, SlipTable, effective_track)


def _variant(**kw):
    v = dict(VARIANT_KEYS)
    v.update(kw)
    return v


def test_zero_is_exact_identity():
    c = CompensationCore(_variant(), wheel_sep=0.125)
    assert c.is_identity
    for vx, wz in [(0.0, 0.0), (0.3, 0.0), (0.0, 1.5), (-0.2, -0.7), (0.123456, 0.987654)]:
        assert c.command(vx, wz, gz=0.4, gz_age=0.01, now=1.0) == (vx, wz)
        assert c.odometry(vx, wz) == wz


def test_feedforward_spin():
    # Table uniforme s = 0,53 (mesure 2026-10-03) : gain 1/(1-s) ~ 2,13.
    c = CompensationCore(_variant(yaw_slip=[0.53] * 9), wheel_sep=0.125)
    vx, wz = c.command(0.0, 1.0)
    assert vx == 0.0
    assert math.isclose(wz, 1.0 / 0.47, rel_tol=1e-9)
    # Odometrie : les roues voient 2,13 rad/s, le robot tourne a 1 rad/s.
    assert math.isclose(c.odometry(0.0, 1.0 / 0.47), 1.0, rel_tol=1e-9)


def test_bilinear_and_clamped():
    t = SlipTable([0.0, 0.4], [0.5, 1.5], [0.2, 0.4,
                                           0.0, 0.2])
    assert math.isclose(t.at(0.0, 1.0), 0.3)
    assert math.isclose(t.at(0.2, 1.0), 0.2)
    assert t.at(0.0, 0.0) == 0.2          # borne sous la 1re colonne
    assert t.at(9.0, 9.0) == 0.2          # borne au-dela du coin


def test_check_rejects_bad_tables():
    assert SlipTable.check([0.0, 0.2], [0.5], [0.1], 0.8) is not None      # taille
    assert SlipTable.check([0.2, 0.0], [0.5], [0.1, 0.1], 0.8) is not None  # ordre
    assert SlipTable.check([0.0], [0.5], [0.9], 0.8) is not None            # > slip_max
    assert SlipTable.check([0.0], [0.5], [-0.1], 0.8) is not None           # < 0
    assert SlipTable.check([0.0], [0.5], [0.5], 0.8) is None


def test_bypass_restores_identity():
    c = CompensationCore(_variant(yaw_slip=[0.5] * 9), wheel_sep=0.125)
    c.bypass = True
    assert c.command(0.0, 1.0) == (0.0, 1.0)
    assert c.odometry(0.0, 2.0) == 2.0


def test_pi_corrects_and_resets():
    c = CompensationCore(_variant(yaw_rate_kp=0.5, yaw_rate_ki=1.0, yaw_rate_i_max=0.2),
                         wheel_sep=0.125)
    _, w = c.command(0.0, 1.0, gz=0.6, gz_age=0.0, now=0.0)
    assert math.isclose(w, 1.0 + 0.5 * 0.4)
    _, w = c.command(0.0, 1.0, gz=0.6, gz_age=0.0, now=0.1)
    assert math.isclose(w, 1.0 + 0.5 * 0.4 + 1.0 * 0.04)
    # Gyro perime : feed-forward seul, integrale remise a zero.
    _, w = c.command(0.0, 1.0, gz=0.6, gz_age=1.0, now=0.2)
    assert w == 1.0 and c.state["gyro_stale"]
    # Ligne droite tenue : w_cmd = 0, v != 0, le robot derive a +0,1 rad/s.
    _, w = c.command(0.3, 0.0, gz=0.1, gz_age=0.0, now=0.3)
    assert math.isclose(w, -0.05)


def test_wheel_saturation_keeps_ratio():
    c = CompensationCore(_variant(yaw_slip=[0.5] * 9), max_wheel_mps=0.5, wheel_sep=0.125)
    vx, wz = c.command(0.5, 1.0)                  # w' = 2 -> roue droite 0,625 m/s
    assert math.isclose(max(abs(vx - wz * 0.0625), abs(vx + wz * 0.0625)), 0.5)
    assert math.isclose(wz / vx, 2.0 / 0.5)


def test_effective_track():
    assert math.isclose(effective_track(0.125, 0.53), 0.125 / 0.47)
