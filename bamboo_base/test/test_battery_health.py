"""Estimateur de sante batterie, sur des series SYNTHETIQUES (sans ROS)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_base.battery_health import BatteryHealth, ChargerDetector, soc_from_rest  # noqa: E402,E501


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


def _rest(det, t0, t1, volt, events, cur=None, since=200.0):
    """Repos continu de t0 a t1 (s), un point par 0,25 s ; volt(t) -> tension, cur(t) ->
    courant (None : non mesure). Le repos a commence `since` s avant t0."""
    t = t0
    while t < t1:
        ev = det.observe(t, volt(t), t - t0 + since, cur(t) if cur else None)
        if ev:
            events.append((t, ev[0]))
        t += 0.25


def test_charger_plug_and_unplug_steps():
    det, ev = ChargerDetector(), []
    # +0,11 V au branchement (mesure du 2026-10-12) ; -0,30 V au debranchement (idem).
    _rest(det, 0, 300, lambda t: 11.91 if t < 100 else (12.02 if t < 200 else 11.72), ev)
    assert [e[1] for e in ev] == ["branche", "debranche"]
    assert 100 < ev[0][0] < 110 and 200 < ev[1][0] < 210          # detecte en < 10 s
    assert det.charging is False


def test_relaxation_after_driving_is_not_a_charger():
    """Apres avoir roule, la tension REMONTE seule (constante de temps ~15 s)."""
    det, ev, t = ChargerDetector(), [], 0.0
    while t < 400:
        v = 11.90 - 0.30 * math.exp(-t / 15.0)
        e = det.observe(t, v, t)                                 # repos qui commence a t = 0
        if e:
            ev.append(e[0])
        t += 0.25
    assert ev == []


def test_charger_already_plugged_seen_by_slope():
    det, ev = ChargerDetector(), []
    _rest(det, 0, 600, lambda t: 11.90 + 0.02 * t / 60.0, ev)     # +0,02 V/min, sans saut
    assert ev and ev[0][1] == "branche"


def test_weak_noisy_charge_is_seen_by_slope():
    """2026-10-13 (soir) : +0,02 V/min, tension bruitee +-0,02 V (charge sur la base)."""
    import random
    rnd = random.Random(3)
    det, ev = ChargerDetector(), []
    _rest(det, 0, 1200, lambda t: 11.88 + 0.02 * t / 60.0 + rnd.uniform(-0.02, 0.02), ev,
          cur=lambda t: -0.66)
    assert ev and ev[0][1] == "branche"


def test_plug_soon_after_parking_is_seen():
    """Gare, puis branche 40 s plus tard : +0,5 V d'un coup pendant la relaxation."""
    det, ev = ChargerDetector(), []
    _rest(det, 0, 120, lambda t: 11.40 - 0.10 * math.exp(-t / 15.0) + (0.5 if t >= 40 else 0),
          ev, since=0.0)
    assert [e[1] for e in ev] == ["branche"]


def test_slow_relaxation_of_a_low_pack_is_not_a_charger():
    """2026-10-13, chargeur DEBRANCHE : pack bas (11,2 V) qui remonte encore de +0,011 V/min
    plusieurs minutes apres avoir roule -- mais de moins en moins vite."""
    det, ev = ChargerDetector(), []
    _rest(det, 0, 1500, lambda t: 11.30 - 0.15 * math.exp(-t / 600.0), ev, since=0.0)
    assert ev == []


def test_load_step_is_not_a_charger():
    """Le lidar demarre au repos : -0,19 V et +0,3 A d'un coup (2026-10-13). Puis il s'arrete."""
    det, ev = ChargerDetector(), []
    _rest(det, 0, 300, lambda t: 11.65 if t < 100 or t >= 200 else 11.46, ev,
          cur=lambda t: -0.60 if t < 100 or t >= 200 else -0.90)
    assert ev == []


def test_charger_step_with_unchanged_current_is_seen():
    det, ev = ChargerDetector(), []
    _rest(det, 0, 200, lambda t: 11.65 if t < 100 else 11.99, ev, cur=lambda t: -0.60)
    assert [e[1] for e in ev] == ["branche"]


def test_motion_resets_the_reference():
    det = ChargerDetector()
    _rest(det, 0, 100, lambda t: 11.9, [])
    assert det.observe(100.5, 11.5, None) is None                  # en roulant : ignore
    assert det.observe(101.0, 12.1, 0.5) is None                   # repos trop court : rien


def test_settling_after_plug_is_not_an_unplug():
    """Mesure du 2026-10-12 : saut +0,7 V au branchement, puis la tension redescend un peu."""
    det, ev = ChargerDetector(), []
    _rest(det, 0, 260,
          lambda t: 11.31 if t < 100 else 12.01 - 0.05 * (1 - math.exp(-(t - 100) / 20.0)), ev)
    assert [e[1] for e in ev] == ["branche"]
    assert det.charging is True
