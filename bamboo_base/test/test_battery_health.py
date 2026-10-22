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


# --- JAUGE (2026-10-17) : 83 % branche / 61 % debranche, puis 50 % sans activite -----------------

def _rest_at(h, v0, i, t0, n, r_ohm=0.2):
    """Repos a courant constant i (A) : V = v0 - R*i, un point par 0,25 s."""
    for k in range(n):
        h.observe(t0 + 0.25 * k, v0 - r_ohm * i, i, [0.0, 0.0], [0.0, 0.0])
    return t0 + 0.25 * n


def test_ocv_compensates_the_internal_drop():
    """Meme pack (OCV 11,80 V), robot qui tire 0,6 A puis 1,4 A : meme pourcentage."""
    a, b = BatteryHealth(r_default_ohm=0.2), BatteryHealth(r_default_ohm=0.2)
    _rest_at(a, 11.80, 0.6, 0.0, 400)
    _rest_at(b, 11.80, 1.4, 0.0, 400)
    assert math.isclose(a.ocv, 11.80, abs_tol=0.01) and math.isclose(b.ocv, 11.80, abs_tol=0.01)
    assert math.isclose(a.soc_raw, b.soc_raw, abs_tol=0.5)
    # sans compensation (tension brute), l'ecart serait de 0,16 V, soit ~7 points de %
    assert abs(soc_from_rest(11.80 - 0.2 * 0.6, 3) - soc_from_rest(11.80 - 0.2 * 1.4, 3)) > 5


def test_no_ocv_while_the_current_moves():
    """Un demarrage de noeud (courant qui bouge) ne doit pas fabriquer un repos."""
    h = BatteryHealth()
    for k in range(400):
        i = 0.6 + (0.5 if (k // 4) % 2 else 0.0)
        h.observe(0.25 * k, 11.8 - 0.2 * i, i, [0.0, 0.0], [0.0, 0.0])
    assert h.has_current and not h._ocv   # courant qui bouge de 0,5 A : refuse


def test_no_percentage_while_charger_plugged():
    h = BatteryHealth()
    _rest_at(h, 12.07, 0.6, 0.0, 400, r_ohm=0.0)
    assert h.display_soc(100.0) is not None            # chargeur inconnu : on affiche
    h.charger.charging = True
    assert h.display_soc(101.0) is None                # branche : pas de pourcentage


def test_display_never_rises_without_a_charger():
    """Relaxation : +0,08 V en 2 min (ni chargeur ni saut) ; le pourcentage ne monte pas."""
    h = BatteryHealth(slew_pct_min=1.0)
    t = _rest_at(h, 11.80, 0.6, 0.0, 400)
    first = h.display_soc(t)
    for k in range(4):
        t = _rest_at(h, 11.80 + 0.02 * (k + 1), 0.6, t, 120)
    assert h.charger.charging is None
    assert h.display_soc(t) <= first


def test_display_falls_at_a_limited_slope():
    """Petite baisse (5 points) : lissee a 1 point par minute."""
    h = BatteryHealth(slew_pct_min=1.0)
    t = _rest_at(h, 11.90, 0.6, 0.0, 400)
    high = h.display_soc(t)
    t2 = _rest_at(h, 11.84, 0.6, t, 400)               # ~ -5 points
    low = h.display_soc(t2)
    elapsed_min = (t2 - t) / 60.0
    assert 0 < high - low <= 1.0 * elapsed_min + 0.1
    assert low > h.soc_raw


def test_regulated_supply_is_recognised_by_its_flat_voltage():
    """Chargeur 12,6 V : la tension ne suit pas le courant du robot."""
    det, ev = ChargerDetector(), []
    for k in range(400):
        i = 0.6 + (0.5 if (k // 20) % 2 else 0.0)
        e = det.observe_regulation(12.07 + 0.002 * ((k * 7) % 3), i)
        if e:
            ev.append(e)
    assert ev and ev[0][0] == "branche" and det.charging is True


def test_a_battery_alone_is_not_taken_for_a_regulated_supply():
    """Pack seul : la tension descend de R*I quand le courant monte (-0,2 V/A)."""
    det = ChargerDetector()
    for k in range(400):
        i = 0.6 + (0.5 if (k // 20) % 2 else 0.0)
        assert det.observe_regulation(12.00 - 0.2 * i, i) is None
    assert det.charging is None


def test_regulation_needs_a_real_current_step():
    """Courant constant : on ne sait rien, on ne conclut pas."""
    det = ChargerDetector()
    for _ in range(400):
        assert det.observe_regulation(12.07, 0.62) is None
    assert det.charging is None


# --- DETECTION CONTINUE du chargeur (angles morts du 2026-10-17) ---------------------------------

def _jump(det, t0, t1, volt, cur=None, step=0.05):
    """Echantillons de t0 a t1 ; volt(t) -> V, cur(t) -> A. Rend [(t, etat)] des evenements."""
    out, t = [], t0
    while t < t1:
        e = det.observe_jump(t, volt(t), cur(t) if cur else 0.62)
        if e:
            out.append((round(t, 1), e[0]))
        t += step
    return out


def test_unplug_seen_even_with_a_short_rest():
    """Debranchement -0,47 V (mesure du jour) : vu en quelques secondes, repos ou non."""
    det = ChargerDetector()
    det.charging = True
    ev = _jump(det, 0, 90, lambda t: 12.06 if t < 40 else 11.59)
    assert ev and ev[0][1] == "debranche" and ev[0][0] < 48.0
    assert det.charging is False


def test_small_plug_step_is_seen():
    """+0,20 V : manque par le detecteur a fenetres (seuil 0,25 V avant 90 s de repos)."""
    det = ChargerDetector()
    ev = _jump(det, 0, 90, lambda t: 11.60 if t < 40 else 11.80)
    assert ev and ev[0][1] == "branche"


def test_plug_during_driving_is_seen():
    """Pas de repos continu : la traction bouge, le courant du robot reste a peu pres plat."""
    det = ChargerDetector()
    ev = _jump(det, 0, 90, lambda t: 11.60 if t < 40 else 11.95,
               cur=lambda t: 1.4 + 0.03 * ((int(t) * 7) % 3))
    assert ev and ev[0][1] == "branche"


def test_load_step_is_not_a_charger_continuous():
    """Un noeud qui demarre : la tension tombe ET le courant monte du meme coup."""
    det = ChargerDetector()
    ev = _jump(det, 0, 90, lambda t: 11.90 if t < 40 else 11.62,
               cur=lambda t: 0.62 if t < 40 else 0.95)
    assert ev == [] and det.charging is None


def test_voltage_spike_without_plateau_is_ignored():
    """Un pic de 2 s qui retombe n'est ni un branchement ni un debranchement."""
    det = ChargerDetector()
    ev = _jump(det, 0, 90, lambda t: 11.60 + (0.40 if 40 <= t < 42 else 0.0))
    assert ev == []


def test_gauge_snaps_to_the_measure_on_a_big_gap():
    """76 % affiches pour 48 % mesures : l'affichage s'aligne, sans descendre a 1 point/min."""
    h = BatteryHealth(slew_pct_min=1.0)
    t = _rest_at(h, 12.00, 0.6, 0.0, 400)
    first = h.display_soc(t)
    t = _rest_at(h, 11.40, 0.6, t, 400)
    now = h.display_soc(t)
    assert first - now > 8.0


def test_stale_ocv_from_the_charging_period_is_dropped():
    """2026-10-17 : 85 % affiches (tension a vide 12,18 V de la charge) pour 50 % reels."""
    h = BatteryHealth()
    t = 0.0
    for k in range(1200):                                   # branche : 12,2 V, puis -0,45 V
        t = 0.25 * k
        v = 12.20 if t < 200.0 else 11.75
        h.observe(t, v, 0.62, [0.0, 0.0], [0.0, 0.0])
    assert h.charger.charging is False
    # pack seul : 11,75 V + R*I = 11,87 V a vide ; jamais les 12,2 V de la periode branchee
    assert abs(h.ocv - (11.75 + 0.2 * 0.62)) < 0.03


def test_noisy_current_still_gives_an_ocv():
    """Le courant du robot fluctue de +-0,1 A au repos : ce n'est pas un courant qui bouge."""
    h = BatteryHealth()
    for k in range(400):
        i = 0.62 + 0.08 * (((k * 7) % 5) - 2) / 2.0
        h.observe(0.25 * k, 11.50 - 0.2 * i, i, [0.0, 0.0], [0.0, 0.0])
    assert h._ocv and abs(h.ocv - 11.50) < 0.05


def test_lidar_unplugged_is_not_a_charger():
    """2026-10-17 : retirer le lidar = courant -0,09 A et tension +0,12 V, pris pour un chargeur."""
    det = ChargerDetector()
    ev = _jump(det, 0, 200, lambda t: 11.07 if t < 100 else 11.19,
               cur=lambda t: 0.61 if t < 100 else 0.52)
    assert ev == [] and det.charging is None
    det2, ev2 = ChargerDetector(), []
    _rest(det2, 0, 300, lambda t: 11.07 if t < 100 else 11.19, ev2,
          cur=lambda t: 0.61 if t < 100 else 0.52)
    assert ev2 == [] and det2.charging is None


def test_lidar_plugged_is_not_an_unplug():
    """L'inverse : la charge augmente (+0,09 A), la tension tombe de 0,12 V."""
    det = ChargerDetector()
    det.charging = True
    ev = _jump(det, 0, 200, lambda t: 11.19 if t < 100 else 11.07,
               cur=lambda t: 0.52 if t < 100 else 0.61)
    assert ev == [] and det.charging is True
