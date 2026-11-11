"""Statistiques d'energie et journaux a taille bornee, SANS ROS ni robot."""
import gzip
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_base import energy_stats as es  # noqa: E402


def feed(st, seconds, v=11.0, i=0.65, vx=0.0, wz=0.0, charging=False, soc=None, t0=0.0, hz=5.0):
    n = int(seconds * hz)
    for k in range(n + 1):
        st.update(t0 + k / hz, v, i, vx, wz, charging, soc)
    return t0 + seconds


def test_classify():
    assert es.classify(0.0, 0.0) == "arret"
    assert es.classify(0.0, 0.5) == "rotation"
    assert es.classify(0.2, 0.0) == "ligne"
    assert es.classify(0.2, 0.4) == "arc"
    assert es.classify(-0.2, 0.0) == "ligne"     # marche arriere


def test_current_and_power_per_activity():
    st = es.EnergyStats()
    t = feed(st, 60, i=0.65, vx=0.0)
    t = feed(st, 60, i=1.40, vx=0.2, t0=t + 0.2)
    a = st.snapshot()["activites"]
    assert abs(a["arret"]["i_a"] - 0.65) < 0.01
    assert abs(a["ligne"]["i_a"] - 1.40) < 0.01
    assert abs(a["ligne"]["dist_m"] - 12.0) < 0.5
    assert a["rotation"]["i_a"] is None and a["rotation"]["t_s"] == 0.0


def test_charger_samples_are_not_counted():
    st = es.EnergyStats()
    feed(st, 120, charging=True, i=-0.5)
    s = st.snapshot()
    assert s["charge"] is True and s["segment"] is None
    assert all(x["t_s"] == 0.0 for x in s["activites"].values())
    assert s["autonomie_min"] is None


def test_segment_starts_at_unplug_and_ends_at_replug():
    st = es.EnergyStats()
    t = feed(st, 30, charging=True)
    t = feed(st, 300, charging=False, soc=100.0, t0=t + 0.2, i=0.65)
    seg = st.snapshot()["segment"]
    assert seg is not None and abs(seg["duree_s"] - 300) < 3
    assert abs(seg["ah"] - 0.65 * 300 / 3600) < 0.002
    feed(st, 5, charging=True, t0=t + 0.2)
    sm = st.take_summary()
    assert sm["kind"] == "segment" and abs(sm["duree_s"] - 300) < 3
    assert st.take_summary() is None             # une seule fois


def test_short_segments_are_dropped():
    st = es.EnergyStats()
    t = feed(st, 5, charging=True)
    t = feed(st, 10, charging=False, t0=t + 0.2)
    feed(st, 5, charging=True, t0=t + 0.2)
    assert st.take_summary() is None


def test_observed_capacity_needs_ten_points():
    st = es.EnergyStats()
    t = feed(st, 10, charging=True)
    # 100 % -> 95 % : trop peu pour conclure
    for k in range(600):
        st.update(t + 0.2 + k, 11.0, 0.65, 0, 0, False, 100.0 - 5.0 * k / 600)
    assert st.snapshot()["capacite_obs_ah"] is None
    # 100 % -> 80 % en 1200 s a 0,65 A = 0,2167 Ah pour 20 points -> 1,083 Ah
    st = es.EnergyStats()
    t = feed(st, 10, charging=True)
    for k in range(1200):
        st.update(t + 0.2 + k, 11.0, 0.65, 0, 0, False, 100.0 - 20.0 * k / 1200)
    cap = st.snapshot()["capacite_obs_ah"]
    assert cap is not None and abs(cap - 1.083) < 0.03
    assert st.snapshot()["fiable"] is True


def test_autonomy_from_remaining_capacity():
    st = es.EnergyStats(capacity_ah=0.88)
    t = feed(st, 10, charging=True)
    feed(st, 200, charging=False, soc=50.0, t0=t + 0.2, i=0.66)
    s = st.snapshot()
    # 0,88 Ah x 50 % = 0,44 Ah ; a 0,66 A -> 40 min
    assert abs(s["reste_ah"] - 0.44) < 0.001
    assert 38 <= s["autonomie_min"] <= 42
    assert s["fiable"] is False                  # pas assez long, pas de capacite observee


def test_internal_resistance_needs_current_spread():
    st = es.EnergyStats()
    feed(st, 30, charging=True)
    # courant constant : pas de resistance
    feed(st, 60, v=11.0, i=0.65, t0=31, charging=False)
    assert st.snapshot()["r_ohm"] is None
    # courant qui varie 0,6 -> 2,0 A, V = 11,4 - 0,25 * I
    st = es.EnergyStats()
    feed(st, 5, charging=True)
    for k in range(300):
        i = 0.6 + 1.4 * (k % 10) / 9.0
        st.update(10 + k * 0.2, 11.4 - 0.25 * i, i, 0.2, 0.0, False, 80.0)
    r = st.snapshot()["r_ohm"]
    assert r is not None and abs(r - 0.25) < 0.01


def test_gap_in_samples_is_not_integrated():
    st = es.EnergyStats()
    st.update(0.0, 11.0, 0.65, 0, 0, False, 90.0)
    st.update(60.0, 11.0, 0.65, 0, 0, False, 90.0)         # trou de 60 s : ignore
    assert st.snapshot()["activites"]["arret"]["t_s"] == 0.0


# ---- journaux bornes ----

def _size(d):
    return sum(os.path.getsize(os.path.join(d, n)) for n in os.listdir(d))


def test_rotating_log_rotates_compresses_and_purges():
    with tempfile.TemporaryDirectory() as d:
        log = es.RotatingLog(d, "energie", ["a", "b", "c"], max_bytes=2000, max_total=6000,
                             min_free=0, fsync_s=1e9)
        for k in range(4000):
            assert log.write([k, "x" * 10, 3.14159], now=1000.0 + k)
        log.close()
        names = sorted(os.listdir(d))
        assert any(n.endswith(".csv.gz") for n in names)
        assert not any(n.endswith(".csv") for n in names)     # tout est range a la fermeture
        assert _size(d) <= 6000 + 2000                        # plafond tenu
        first = [n for n in names if n.endswith(".gz")][-1]
        with gzip.open(os.path.join(d, first), "rt") as fh:
            assert fh.readline().strip() == "a,b,c"           # chaque fichier a son en-tete


def test_rotating_log_total_stays_bounded_over_a_long_run():
    with tempfile.TemporaryDirectory() as d:
        log = es.RotatingLog(d, "energie", ["a"], max_bytes=3000, max_total=9000, min_free=0,
                             fsync_s=1e9)
        for k in range(20000):
            log.write([k], now=1.0 + k)
            if k % 2000 == 0:
                assert _size(d) <= 9000 + 3000
        log.close()
        assert _size(d) <= 9000 + 3000


def test_rotating_log_suspends_when_disk_is_nearly_full():
    with tempfile.TemporaryDirectory() as d:
        log = es.RotatingLog(d, "energie", ["a"], min_free=1 << 60)    # jamais assez de libre
        assert log.write([1], now=1.0) is False
        assert log.suspended is True
        assert os.listdir(d) == []                                    # rien d'ecrit


def test_rotating_log_cleans_raw_leftover_at_start():
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, "energie_20260101_000000.csv"), "w") as fh:
            fh.write("a\n1\n")
        es.RotatingLog(d, "energie", ["a"], min_free=0)
        assert os.listdir(d) == ["energie_20260101_000000.csv.gz"]


def test_jsonl_tail_keeps_only_the_last_lines():
    with tempfile.TemporaryDirectory() as d:
        j = es.JsonlTail(os.path.join(d, "segments.jsonl"), keep=5, max_bytes=300)
        for k in range(100):
            j.append({"n": k, "pad": "x" * 20})
        with open(j.path) as fh:
            lines = fh.readlines()
        assert len(lines) <= 12 and '"n": 99' in lines[-1]
        assert os.path.getsize(j.path) < 600


# --- segment impossible (2026-11-10) : 2,69 Ah comptes « sur batterie » pour un pack de 0,88 Ah, jauge a 100 % ------

def test_a_segment_above_the_pack_capacity_without_gauge_movement_is_invalid():
    st = es.EnergyStats(capacity_ah=0.88)
    t = feed(st, 10, charging=True)
    # 0,65 A pendant 2 h = 1,3 Ah > 1,5 x 0,88 = 1,32 ? non : on prend 3 h = 1,95 Ah
    for k in range(3 * 3600):
        st.update(t + 0.2 + k, 12.05, 0.65, 0, 0, False, 100.0)
    s = st.snapshot()
    assert s["segment_invalide"] is True
    assert s["autonomie_min"] is None and s["fiable"] is False
    st.update(t + 0.2 + 3 * 3600 + 1, 12.05, 0.65, 0, 0, True, None)      # le chargeur est enfin vu
    sm = st.take_summary()
    assert sm["invalide"] is True


def test_a_real_discharge_beyond_the_capacity_is_not_flagged():
    """Un pack qui perd 40 points de jauge a bien consomme ce qu'il a consomme : le segment est valide."""
    st = es.EnergyStats(capacity_ah=0.88)
    t = feed(st, 10, charging=True)
    for k in range(3 * 3600):
        st.update(t + 0.2 + k, 11.0, 0.65, 0, 0, False, 100.0 - 40.0 * k / (3 * 3600))
    assert st.snapshot()["segment_invalide"] is False


def test_a_normal_short_segment_is_valid():
    st = es.EnergyStats(capacity_ah=0.88)
    t = feed(st, 10, charging=True)
    feed(st, 600, charging=False, soc=100.0, t0=t + 0.2, i=0.66)
    s = st.snapshot()
    assert s["segment_invalide"] is False and s["autonomie_min"] is not None
