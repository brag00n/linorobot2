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


def _first_order(tau, k=1.0, n=40, dt=0.05, target=50.0):
    t = [i * dt for i in range(n)]
    return t, [target] * n, [target * k * (1 - math.exp(-ti / tau)) for ti in t]


def test_step_metrics_and_best_gains():
    fast, slow = cm.step_metrics(*_first_order(0.2)), cm.step_metrics(*_first_order(0.8))
    assert fast["montee_s"] < slow["montee_s"]
    assert fast["erreur_statique"] < 0.05
    off = cm.step_metrics(*_first_order(0.2, k=0.7))         # n'atteint jamais 90 %
    assert off["montee_s"] is None and off["erreur_statique"] > 0.25
    best, score, table = cm.best_gains({(0.6, 0.3, 0.5): [slow], (1.5, 0.6, 0.5): [fast]}, 2.0)
    assert best == (1.5, 0.6, 0.5) and len(table) == 2
    assert cm.step_metrics([0, 1, 2, 3, 4], [0.1] * 5, [0.1] * 5) is None   # cible nulle


def test_battery_profile():
    cycles = []
    for k in range(10):
        ah = 0.2 * (k + 1)
        v = 12.4 - 0.25 * k
        cycles.append({"ah": ah, "v_repos": v, "v_charge": v - 0.36, "i_charge": 2.6,
                       "i_repos": 0.6})
    cycles[0]["v_repos_initial"] = 12.55
    p = cm.battery_profile(cycles, 3)
    assert math.isclose(p["capacite_ah"], 2.0)
    assert p["profil_v"][-1] == 12.55 and p["profil_v"][0] < p["profil_v"][-1]
    assert math.isclose(p["r_ref_ohm"], 0.18, rel_tol=1e-3)
    txt = cm.battery_candidate_yaml("robot", p, {"v": 12.55, "i": 0.56}, "/tmp/r.json")
    assert "batt_profile_v:" in txt and "batt_r_warn_ohm: 0.2700" in txt
    assert cm.battery_profile(cycles[:2], 3) is None


# --- rampe de vitesse : rotation progressive de la decharge batterie (2026-10-18) -----------------
def test_slew_rises_and_falls_without_a_jump():
    s = cm.Slew(accel=1.0)
    outs, dt = [], 0.1
    for _ in range(40):
        outs.append(s.update(2.5, dt))                    # montee vers 2,5 rad/s
    assert outs[0] <= 0.11 and max(outs) == 2.5
    assert all(b - a <= 0.1 + 1e-9 for a, b in zip(outs, outs[1:]))     # jamais plus de a*dt
    for _ in range(40):
        outs.append(s.update(0.0, dt))                    # descente jusqu'a l'arret
    assert s.stopped and outs[-1] == 0.0
    assert all(abs(b - a) <= 0.1 + 1e-9 for a, b in zip(outs, outs[1:]))


def test_slew_time_to_nominal_matches_accel():
    assert cm.Slew(1.0).time_to(2.5) == 2.5
    assert cm.Slew(0.5).time_to(2.5) == 5.0


def test_slew_bounds_a_late_tick():
    """Un tick en retard (dt = 5 s) ne doit pas produire un saut : dt est borne a 1 s."""
    s = cm.Slew(1.0)
    assert s.update(2.5, 5.0) == 1.0
    assert s.update(-2.5, 5.0) == 0.0                      # puis redescend d'au plus 1 rad/s


def test_slew_reverses_through_zero():
    s = cm.Slew(1.0)
    for _ in range(30):
        s.update(2.0, 0.1)
    seq = [s.update(-2.0, 0.1) for _ in range(50)]
    assert min(seq) == -2.0
    assert all(abs(b - a) <= 0.1 + 1e-9 for a, b in zip(seq, seq[1:]))
