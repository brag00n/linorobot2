"""Calcul de la calibration sur des paliers SYNTHETIQUES (sans ROS ni robot)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_base import calibration_math as cm  # noqa: E402


def _steps(slip_of_w, v=0.0, w_cmds=(0.6, 1.2, 2.0, 3.0), lifted=False, flip=1.0):
    out = []
    for w in w_cmds:
        for sgn in (1.0, -1.0):
            ww = sgn * w
            real = 0.0 if lifted else ww * (1.0 - slip_of_w(abs(ww)))
            gz = [flip * real * (1 + 0.01 * ((i % 3) - 1)) for i in range(30)]
            out.append(cm.step_stats(v, ww, gz, [ww] * 30, [v] * 30))
    return out


def test_uniform_slip_gives_flat_table():
    steps = _steps(lambda w: 0.53)
    faults, _ = cm.check_steps(steps, 0.10)
    assert not faults
    table, measured = cm.build_table(steps, [0.0, 0.2, 0.4], [0.5, 1.0, 1.5], 0.8)
    assert measured == [True, False, False]
    assert all(math.isclose(x, 0.53, abs_tol=1e-3) for x in table)   # lignes recopiees


def test_lifted_robot_is_rejected():
    faults, _ = cm.check_steps(_steps(lambda w: 0.5, lifted=True), 0.10)
    assert faults and "SOULEVE" in faults[0]


def test_inverted_gyro_sign_is_named():
    faults, _ = cm.check_steps(_steps(lambda w: 0.5, flip=-1.0), 0.10)
    assert any("imu_gz_sign" in f for f in faults)


def test_asymmetry_detected():
    steps = _steps(lambda w: 0.5)
    for s in steps:
        if s["w_cmd"] < 0:
            s["s"] = 0.3
    _, faults = cm.asymmetry(steps, 0.15)
    assert faults


def test_table_indexed_on_real_rate_and_zero_fill():
    # s croit avec w : la colonne 1,5 rad/s REELLE n'est atteinte qu'a w_cmd ~3,0.
    steps = _steps(lambda w: 0.4 + 0.05 * w)
    table, _ = cm.build_table(steps, [0.0, 0.2], [0.5, 1.0, 1.5], 0.8, fill="zero")
    assert table[3:] == [0.0, 0.0, 0.0]
    assert table[0] < table[1] < table[2]


def test_rest_check():
    st = cm.rest_stats([0.001] * 50, [0.0, 0.1], 10.0)
    assert cm.rest_ok(st, 0.005, 0.5)[0]
    st = cm.rest_stats([0.02] * 50, [0.0, 0.1], 10.0)
    assert not cm.rest_ok(st, 0.005, 0.5)[0]


def test_candidate_yaml_shape():
    txt = cm.candidate_yaml("essai", "robot", [0.5] * 9, [0.0, 0.2, 0.4], [0.5, 1.0, 1.5],
                            [True, True, False], "/tmp/r.json", 12.3, 0.125)
    assert "      essai:" in txt and "yaw_slip: [" in txt and "RECOPIEE" in txt
