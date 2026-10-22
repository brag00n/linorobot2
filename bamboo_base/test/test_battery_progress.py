"""Avancement de calibration batterie et resistance aux coupures, SANS ROS ni robot."""
import json
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_base import battery_progress as bp  # noqa: E402

# Decharge REELLE du 2026-10-17 (decharge_20261017_220736.csv, 55 min, 0,587 Ah, 0,64 A) :
# (minutes depuis le debranchement, tension du pack). Mediane sur 30 s, releves a la main.
REAL = [(0, 11.563), (1, 11.430), (2, 11.374), (3, 11.294), (5, 11.237), (8, 11.142),
        (10, 11.090), (15, 10.986), (20, 10.882), (30, 10.702), (40, 10.502), (45, 10.306),
        (50, 9.851), (53, 9.191), (54, 8.732)]
I_MEAN = 0.64


def v_at(minute):
    for (m0, v0), (m1, v1) in zip(REAL, REAL[1:]):
        if m0 <= minute <= m1:
            return v0 + (v1 - v0) * (minute - m0) / (m1 - m0)
    return REAL[-1][1]


def run(prog, minutes=54.5, step=1.0, i=I_MEAN, resting=True):
    """Rejoue la decharge reelle a 1 Hz (par defaut) ; rend [(t, snapshot)] et l'arret."""
    out, t = [], 0.0
    stop = ""
    while t <= minutes * 60.0:
        stop = prog.update(t, v_at(t / 60.0) - 0.0, i, resting) or stop
        out.append((t, prog.snapshot("decharge")))
        if stop:
            break
        t += step
    return out, stop


def fresh():
    p = bp.BattProgress()
    p.set_start(11.563, I_MEAN)
    return p


def test_stops_at_the_knee_and_reads_the_capacity():
    p = fresh()
    out, stop = run(p)
    assert stop.startswith("coude atteint")
    t_stop = out[-1][0] / 60.0
    assert 52.0 < t_stop < 54.5                       # 9,3 V est atteint vers 53 min
    # Au coude, la capacite mesuree est celle de la decharge (0,587 Ah au total).
    assert 0.50 < p.ah < 0.62


def test_quality_of_a_weak_pack_is_low():
    p = fresh()
    run(p, minutes=30.0)
    assert p.capacity_ah is not None
    assert 0.35 < p.capacity_ah < 0.9                  # pack de ~0,6 Ah
    assert p.quality is not None and p.quality < 50.0  # ~23 % de la reference 2,6 Ah
    assert bp.quality_label(p.quality) == "faible"


def test_no_announcement_before_ten_percent():
    p = fresh()
    out, _ = run(p, minutes=2.0)
    assert p.capacity_ah is None and p.quality is None
    assert math.isnan(out[-1][1]["capacity_ah"]) and math.isnan(out[-1][1]["remaining_s"])


def test_remaining_time_is_within_20_percent_at_half():
    p = fresh()
    run(p, minutes=30.0)                               # ~47 % restant
    assert p.remaining_s is not None
    real_left = (53.0 - 30.0) * 60.0                   # jusqu'au coude, en secondes
    assert abs(p.remaining_s - real_left) < 0.35 * real_left


def test_zero_current_never_divides():
    p = bp.BattProgress()
    p.set_start(12.0, 0.0)
    for k in range(400):
        p.update(float(k), 12.0, 0.0)
    assert p.remaining_s is None and p.stop_reason == ""


def test_charger_left_plugged_aborts_without_progress():
    p = bp.BattProgress(no_progress_s=600.0)
    p.set_start(12.1, 0.6)
    stop = ""
    for k in range(1000):
        stop = p.update(float(k), 12.08 + 0.01 * ((k * 7) % 3), 0.6) or stop
        if stop:
            break
    assert "ne se decharge pas" in stop


def test_a_good_pack_scores_high():
    p = bp.BattProgress(ref_capacity_ah=2.6)
    assert bp.quality_pct(2.6, 0.15, 2.6, 0.15, None) == 100.0
    assert bp.quality_pct(2.6, None, 2.6, 0.15, None) == 100.0       # R inconnue : capacite seule
    assert bp.quality_pct(3.5, 0.10, 2.6, 0.15, None) == 100.0       # plafonne a 100
    assert 40.0 < bp.quality_pct(1.3, 0.30, 2.6, 0.15, None) < 55.0
    assert p.quality is None


def test_measured_resistance_replaces_the_estimate():
    p = fresh()
    assert p.r_estimated and p.r_ohm == 0.15
    p.set_resistance(0.31)
    assert not p.r_estimated and p.r_ohm == 0.31
    p.set_resistance(9.0)                                            # absurde : ignoree
    assert p.r_ohm == 0.31


def test_snapshot_marks_unknown_as_nan_and_serializes():
    p = fresh()
    snap = p.snapshot("decharge")
    assert snap["capacity_ref_ah"] == 2.6 and math.isnan(snap["capacity_ah"])
    json.dumps(bp.clean(snap), allow_nan=False)                      # sans NaN


# ----------------------------------------------------------------------- coupures
class Clock:
    def __init__(self, t=1_760_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def test_journal_survives_a_hard_cut():
    """Processus tue en pleine decharge : le journal contient toutes les secondes ecrites."""
    folder, clk = tempfile.mkdtemp(), Clock()
    j = bp.BattJournal(folder, "batterie_decharge", state_every_s=5.0, clock=clk)
    p = fresh()
    for k in range(400):
        clk.t += 1.0
        p.update(float(k), v_at(k / 60.0), I_MEAN)
        j.log(p, "decharge")
    j.session["en_cours"] = True
    # PAS de close() : on simule la coupure. Les fichiers doivent etre lisibles tels quels.
    with open(j.path, encoding="utf-8") as fh:
        assert sum(1 for _ in fh) == 401                              # entete + 400 lignes
    with open(j.session_path, encoding="utf-8") as fh:
        assert json.load(fh)["en_cours"] is True                      # jamais a moitie ecrit
    res = bp.recover(folder, min_rows=60)
    assert res and res["partielle"] is True
    assert res["raison_arret"].startswith("interrompue")
    assert res["ah"] > 0
    # Apres reprise, l'etat n'est plus « en cours » : pas de seconde reprise.
    assert bp.recover(folder) is None
    assert bp.read_last(folder)["partielle"] is True


def test_recover_tolerates_a_truncated_last_line():
    folder, clk = tempfile.mkdtemp(), Clock()
    j = bp.BattJournal(folder, "batterie_decharge", clock=clk)
    p = fresh()
    for k in range(120):
        clk.t += 1.0
        p.update(float(k), v_at(k / 60.0), I_MEAN)
        j.log(p, "decharge")
    j._fh.write("1760000121.0,decharge,11.4")                        # coupure en pleine ligne
    j._fh.flush()
    res = bp.recover(folder, min_rows=60)
    assert res is not None and res["partielle"]


def test_recover_ignores_a_session_already_closed():
    folder, clk = tempfile.mkdtemp(), Clock()
    j = bp.BattJournal(folder, "batterie_decharge", clock=clk)
    p = fresh()
    for k in range(100):
        clk.t += 1.0
        p.update(float(k), v_at(k / 60.0), I_MEAN)
        j.log(p, "decharge")
    j.close(p, "coude atteint", partial=False)
    assert bp.recover(folder) is None
    last = bp.read_last(folder)
    assert last["partielle"] is False and last["raison_arret"] == "coude atteint"


def test_atomic_write_leaves_no_temporary_file_and_keeps_the_old_one_on_failure():
    folder = tempfile.mkdtemp()
    path = os.path.join(folder, "etat.json")
    bp.atomic_write_json(path, {"a": 1})
    assert os.listdir(folder) == ["etat.json"]

    class Bad:
        pass

    try:
        bp.atomic_write_json(path, {"a": float("nan")})               # allow_nan=False
    except ValueError:
        pass
    with open(path, encoding="utf-8") as fh:
        assert json.load(fh) == {"a": 1}                              # ancien fichier intact


def test_no_previous_result_gives_none():
    assert bp.read_last(tempfile.mkdtemp()) is None


# ----------------------------------------------------------------------- types de pack
TYPES_FILE = os.path.join(os.path.dirname(__file__), "..", "config", "battery_types.yaml")


def test_battery_types_file_defines_both_robots():
    types = bp.load_battery_types(TYPES_FILE)
    assert {"esp32_ups_3s1p", "stm32_3s2p_7ah"} <= set(types)
    for t in types.values():
        assert t["cells"] == 3 and t["capacity_ah"] > 0 and t["r_ohm"] > 0
        assert t["method"] in ("ah_measured", "time_assumed")
        assert t["knee_cell_v"] > t["load_knee_cell_v"] - 0.5
        assert t["curve"][0][1] == 0.0 and t["curve"][-1][1] == 1.0
    # le STM32 ne s'arrete JAMAIS sous l'alarme de la carte (9,6 V)
    stm = types["stm32_3s2p_7ah"]
    assert stm["knee_cell_v"] * stm["cells"] > stm["alarm_v"]
    assert stm["motor_calibration"] is False
    assert types["esp32_ups_3s1p"]["motor_calibration"] is True


def test_progress_follows_the_robot_pack_type():
    types = bp.load_battery_types(TYPES_FILE)
    esp = bp.progress_for_type(types, "esp32_ups_3s1p")
    stm = bp.progress_for_type(types, "stm32_3s2p_7ah")
    assert esp.ref_capacity_ah == 2.6 and stm.ref_capacity_ah == 7.0
    assert esp.method == "ah_measured" and stm.method == "time_assumed"
    assert abs(esp.cutoff_v - 9.3) < 1e-6 and abs(stm.cutoff_v - 9.9) < 1e-6
    # type inconnu ou vide : repli sur le premier, nomme dans type_name
    assert bp.progress_for_type(types, "").type_name == next(iter(types))
    assert bp.progress_for_type(types, "inconnu").type_name == next(iter(types))


def test_stm32_without_current_integrates_the_assumed_background_current():
    """Pas d'INA219 : courant None. La capacite vient de la duree x courant suppose."""
    types = bp.load_battery_types(TYPES_FILE)
    p = bp.progress_for_type(types, "stm32_3s2p_7ah")
    p.set_start(12.1, None)
    i_assumed = types["stm32_3s2p_7ah"]["assumed_current_a"]
    for k in range(0, 3600):                                  # 1 h de decharge reguliere
        p.update(float(k), 12.1 - 0.0003 * k, None)
    assert p.current_assumed
    assert abs(p.ah - i_assumed * 1.0) < 0.02                  # 1 h x courant suppose
    snap = p.snapshot("decharge")
    assert snap["current_assumed"] is True and snap["r_estimated"] is True


def test_esp32_with_measured_current_is_not_marked_assumed():
    types = bp.load_battery_types(TYPES_FILE)
    p = bp.progress_for_type(types, "esp32_ups_3s1p")
    p.set_start(11.563, I_MEAN)
    run(p, minutes=5.0)
    assert not p.current_assumed


def test_a_good_stm32_pack_scores_high_and_a_weak_one_low():
    assert bp.quality_pct(7.0, 0.075, 7.0, 0.075) == 100.0
    assert bp.quality_pct(3.5, 0.15, 7.0, 0.075) < 60.0       # moitie de capacite, R double


def test_remaining_is_given_provisionally_before_the_estimate_is_stable():
    p = fresh()
    run(p, minutes=12.0)                               # capacite connue, encore en rattrapage
    assert p.capacity_ah is not None
    assert p.remaining_s is not None and p.remaining_s > 0.0


def test_an_isolated_jump_does_not_hide_the_remaining_time():
    p = fresh()
    run(p, minutes=30.0)
    t = p.t_last
    p._cap_hist.append((t, p.capacity_ah * 1.4))       # un echantillon aberrant
    assert p._remaining() is not None
