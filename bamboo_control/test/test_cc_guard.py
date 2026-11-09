"""Coeur du garde-fou du Command Center, sans ROS."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_control.cc_guard_node import GuardCore  # noqa: E402


def beat(g, t, video=True, tab="A"):
    return g.heartbeat(tab, video, t)


def test_sleeps_when_nothing_moves():
    g = GuardCore()
    beat(g, 0.0)
    assert g.evaluate(10.0, moving=False) is None     # CC silencieux depuis 10 s, mais rien ne bouge


def test_never_seen_cc_does_not_trip_a_gamepad_session():
    g = GuardCore()
    # Robot arme a la manette, CC jamais ouvert : cas normal, aucun arret.
    assert g.evaluate(0.0, moving=True) is None
    assert g.evaluate(60.0, moving=True) is None
    assert g.cc_state(60.0) == "none"


def test_closed_cc_is_forgotten_so_a_later_gamepad_arming_is_not_tripped():
    g = GuardCore()
    beat(g, 0.0)                                      # un CC a existe, puis il est ferme, rien d'arme
    assert g.evaluate(5.0, moving=False) is None      # oublie
    assert g.cc_state(5.0) == "none"
    assert g.evaluate(6.0, moving=True) is None       # armement a la manette : aucun arret
    assert g.evaluate(60.0, moving=True) is None


def test_heartbeat_silence_trips_after_1_5_seconds():
    g = GuardCore()
    beat(g, 0.0)
    assert g.evaluate(1.4, moving=True) is None       # 1,4 s : on tolere
    cause = g.evaluate(1.6, moving=True)              # 1,6 s : arret
    assert cause and "CC silencieux" in cause
    assert g.evaluate(1.7, moving=True) is None       # une seule fois par perte, pas de rafale
    assert g.evaluate(5.0, moving=True) is None


def test_video_lost_trips_after_3_seconds_while_heartbeat_is_fine():
    g = GuardCore()
    beat(g, 0.0, video=True)
    beat(g, 1.0, video=False)                         # la video devient mauvaise a t = 1,0 s
    for t in (1.5, 2.0, 2.5, 3.0, 3.5, 3.9):          # le battement reste regulier
        beat(g, t, video=False)
        assert g.evaluate(t, moving=True) is None     # moins de 3 s de video mauvaise : on tolere
    beat(g, 4.1, video=False)                         # 3,1 s de video mauvaise
    cause = g.evaluate(4.1, moving=True)
    assert cause and "video perdue" in cause


def test_video_recovering_before_the_limit_cancels_the_countdown():
    g = GuardCore()
    beat(g, 0.0, video=False)
    beat(g, 2.0, video=False)
    beat(g, 2.5, video=True)                          # revenue a 2,5 s
    beat(g, 4.0, video=True)
    assert g.evaluate(4.0, moving=True) is None
    assert g.video_state(4.0) == "ok"


def test_second_loss_trips_again_after_recovery():
    g = GuardCore()
    beat(g, 0.0)
    assert g.evaluate(2.0, moving=True)               # perte 1
    beat(g, 3.0)                                      # le CC revient
    assert g.evaluate(3.0, moving=True) is None       # sain : on se reinitialise
    assert g.evaluate(5.0, moving=True)               # perte 2 : un nouvel arret


def test_second_tab_cannot_hijack_a_live_pilot():
    g = GuardCore()
    assert beat(g, 0.0, tab="A")
    assert not beat(g, 0.4, tab="B")                  # A est vivant : B n'est pas le pilote
    assert g.pilot == "A" and len(g.others) == 1
    beat(g, 0.5, tab="A")
    assert g.evaluate(0.6, moving=True) is None


def test_second_tab_takes_over_once_the_pilot_is_dead():
    g = GuardCore()
    beat(g, 0.0, tab="A")
    assert not beat(g, 1.0, tab="B")                  # A silencieux depuis 1,0 s : encore pilote
    assert beat(g, 1.8, tab="B")                      # A mort depuis 1,8 s : B prend la place
    assert g.pilot == "B"
    assert g.cc_state(1.9) == "ok"


def test_link_ok_requires_a_connected_video():
    g = GuardCore()
    beat(g, 0.0, video=True)
    assert g.link_ok(0.1)
    beat(g, 0.5, video=False)
    assert not g.link_ok(0.6)                         # pas de pilotage sur une video morte, meme avant 3 s
    assert g.video_state(0.6) == "bad"


def test_snapshot_reports_the_cause_for_the_beacon():
    g = GuardCore()
    beat(g, 0.0)
    g.evaluate(2.0, moving=True)
    s = g.snapshot(2.0, moving=True, armed=True)
    assert s["cc"] == "lost" and s["tripped"] and "CC silencieux" in s["reason"]
    assert s["age_s"] == 2.0 and s["pilot"] == "A"


def test_garbage_does_not_crash():
    g = GuardCore()
    assert g.heartbeat(None, None, 0.0)               # id absent, video absente : accepte sans lever
    assert g.cc_state(0.1) == "ok" and not g.link_ok(0.1)


def test_silence_cause_says_the_tab_was_hidden():
    g = GuardCore()
    g.heartbeat("A", True, 0.0, hidden=False, dt_ms=500, seq=1)
    g.heartbeat("A", True, 1.0, hidden=True, dt_ms=1000, seq=2)      # l'onglet passe en arriere-plan
    cause = g.evaluate(3.0, moving=True)
    assert cause and "MASQUE" in cause and "CC silencieux" in cause


def test_silence_cause_says_messages_were_lost():
    g = GuardCore()
    g.heartbeat("A", True, 0.0, hidden=False, dt_ms=500, seq=1)
    g.heartbeat("A", True, 0.5, hidden=False, dt_ms=500, seq=5)      # 3 numeros manquants : perdus en route
    cause = g.evaluate(2.5, moving=True)
    assert cause and "3 battement(s) PERDU(S)" in cause and "visible" in cause


def test_silence_cause_says_the_page_was_slow_not_lossy():
    g = GuardCore()
    g.heartbeat("A", True, 0.0, hidden=False, dt_ms=500, seq=1)
    g.heartbeat("A", True, 0.6, hidden=False, dt_ms=1900, seq=2)     # envois espaces, aucun numero saute
    cause = g.evaluate(2.4, moving=True)
    assert cause and "page ralentie" in cause and "1900" in cause and "PERDU" not in cause


def test_diagnostic_counters_restart_after_a_trip():
    g = GuardCore()
    g.heartbeat("A", True, 0.0, hidden=False, dt_ms=500, seq=1)
    g.heartbeat("A", True, 0.5, hidden=False, dt_ms=500, seq=4)
    assert "PERDU" in g.evaluate(2.5, moving=True)
    g.heartbeat("A", True, 3.0, hidden=False, dt_ms=500, seq=5)
    g.evaluate(3.0, moving=True)
    cause = g.evaluate(5.0, moving=True)
    assert cause and "PERDU" not in cause                           # la perte precedente n'est pas re-imputee


def test_old_heartbeats_without_diagnostic_fields_still_work():
    g = GuardCore()
    assert g.heartbeat("A", True, 0.0)                              # ancienne page : ni hidden, ni dt, ni seq
    cause = g.evaluate(2.0, moving=True)
    assert cause and "CC silencieux" in cause
