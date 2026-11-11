"""Moteur de missions : composition, prerequis, securite, delais, annulation. SANS ROS ni robot."""
import copy
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_control import mission_core as mc  # noqa: E402

CALL = lambda s, **a: {"service": s, "args": a}   # noqa: E731

MODEL = {
    "capacites": {
        "pile": {"demarre": CALL("/nav_stack", data=True), "pret_quand": {"noeud": "/nav_supervisor"},
                 "delai_s": 90, "annule": CALL("/nav_stack", data=False)},
        "mode_nav": {"exige": ["pile"], "lance": CALL("/cmd", command="navigation"),
                     "fini_quand": {"nav.submode": ["navigation"]}, "delai_s": 60},
        "reloc": {"exige": ["pile", "mode_nav"], "lance": CALL("/cmd", command="relocalize"),
                  "fini_quand": {"nav.loc_state": ["ok"]}, "delai_s": 120},
        "goto": {"exige": ["reloc"], "arme": True,
                 "lance": CALL("/cmd", command="goto", arg="$destination"),
                 "fini_quand": {"nav.goal_status": ["succeeded"]}, "delai_s": 300,
                 "annule": CALL("/cmd", command="cancel_all")},
    },
    "missions": {
        "Prepa": {"etapes": ["pile", "reloc"]},
        "Aller": {"etapes": ["goto"], "parametres": ["destination"]},
    },
}

ST0 = {"noeuds": [], "nav.submode": "idle", "nav.loc_state": "inconnue", "nav.goal_status": ""}


def eng():
    return mc.MissionEngine(copy.deepcopy(MODEL))


def run(e, t, st, link=True, armed=False, steps=1, dt=1.0):
    """Avance de `steps` ticks ; rend (actions de tous les ticks, t)."""
    acts = []
    for _ in range(steps):
        t += dt
        acts += e.tick(t, st, link, armed)
    return acts, t


# ---- modele ----

def test_compose_adds_prerequisites_in_order_without_duplicates():
    assert mc.compose(MODEL, "Prepa") == ["pile", "mode_nav", "reloc"]
    assert mc.compose(MODEL, "Aller") == ["pile", "mode_nav", "reloc", "goto"]


def test_validate_reports_every_defect():
    bad = copy.deepcopy(MODEL)
    bad["capacites"]["reloc"]["exige"] = ["inconnue"]
    bad["missions"]["X"] = {"etapes": []}
    bad["missions"]["Y"] = {"etapes": ["nexiste_pas"]}
    with pytest.raises(mc.MissionError) as ei:
        mc.validate(bad)
    msg = str(ei.value)
    assert "prerequis inconnu inconnue" in msg and "aucune etape" in msg and "capacite inconnue nexiste_pas" in msg


def test_validate_detects_a_prerequisite_cycle():
    bad = copy.deepcopy(MODEL)
    bad["capacites"]["pile"]["exige"] = ["goto"]
    with pytest.raises(mc.MissionError, match="cycle"):
        mc.validate(bad)


def test_the_shipped_model_is_valid():
    import yaml
    p = os.path.join(os.path.dirname(__file__), "..", "config", "missions.yaml")
    model = yaml.safe_load(open(p, encoding="utf-8"))
    mc.validate(model)
    for m in model["missions"]:
        assert mc.compose(model, m)


# ---- preflight : refus AVANT tout effet ----

def test_a_moving_mission_is_refused_without_arming():
    e = eng()
    ok, why, acts = e.start("Aller", {"destination": "p1"}, 0.0, ST0, link_ok=True, armed=False)
    assert not ok and "armee" in why and acts == [] and e.status == mc.IDLE


def test_a_moving_mission_is_refused_without_the_cc_link():
    e = eng()
    ok, why, _ = e.start("Aller", {"destination": "p1"}, 0.0, ST0, link_ok=False, armed=True)
    assert not ok and "liaison" in why


def test_a_missing_parameter_is_refused():
    e = eng()
    ok, why, _ = e.start("Aller", {}, 0.0, ST0, True, True)
    assert not ok and "destination" in why


def test_a_still_mission_needs_neither_link_nor_arming():
    e = eng()
    ok, _, _ = e.start("Prepa", {}, 0.0, ST0, link_ok=False, armed=False)
    assert ok


def test_a_second_mission_is_refused_while_one_runs():
    e = eng()
    e.start("Prepa", {}, 0.0, ST0, True, False)
    ok, why, _ = e.start("Prepa", {}, 1.0, ST0, True, False)
    assert not ok and "deja en cours" in why


# ---- deroulement nominal ----

def test_nominal_run_calls_in_order_and_finishes():
    e = eng()
    st = dict(ST0)
    e.start("Prepa", {}, 0.0, st, True, False)
    acts, t = run(e, 0.0, st)
    assert [a["call"] for a in acts] == ["/nav_stack"] and acts[0]["args"] == {"data": True}
    # la pile n'est pas prete : rien ne part
    acts, t = run(e, t, st, steps=3)
    assert acts == [] and e.current() == "pile"
    st["noeuds"] = ["/nav_supervisor"]
    acts, t = run(e, t, st)
    assert e.current() == "mode_nav"
    acts, t = run(e, t, st, steps=2)                       # PENDING -> lance
    assert any(a["args"].get("command") == "navigation" for a in acts)
    st["nav.submode"] = "navigation"
    acts, t = run(e, t, st, steps=3)
    assert e.current() == "reloc"
    more, t = run(e, t, st, steps=4)                       # PENDING -> WAITING_READY -> lance
    assert any(a["args"].get("command") == "relocalize" for a in acts + more)
    st["nav.loc_state"] = "ok"
    run(e, t, st, steps=4)
    assert e.status == mc.DONE and e.snapshot()["etapes"][-1]["etat"] == "done"


def test_a_condition_already_true_does_not_end_a_step_before_it_was_launched():
    """Une relocalisation deja « ok » d'avant la mission ne doit pas la faire sauter sans rien lancer."""
    e = eng()
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "ok"}
    e.start("Prepa", {}, 0.0, st, True, False)
    acts, t = run(e, 0.0, st, steps=12)
    assert any(a["args"].get("command") == "relocalize" for a in acts)
    assert e.status == mc.DONE


def test_destination_parameter_is_substituted():
    e = eng()
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "ok",
          "nav.goal_status": ""}
    e.start("Aller", {"destination": "p3"}, 0.0, st, True, True)
    acts, t = run(e, 0.0, st, steps=14, armed=True)
    gotos = [a for a in acts if a["args"].get("command") == "goto"]
    assert gotos and gotos[0]["args"]["arg"] == "p3"


# ---- securite en cours de route ----

def _moving(e):
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "ok",
          "nav.goal_status": "active"}
    e.start("Aller", {"destination": "p1"}, 0.0, st, True, True)
    acts, t = run(e, 0.0, st, steps=14, armed=True)
    assert e.current() == "goto" and e.step_state == mc.RUNNING_STEP
    return st, t


def test_losing_the_cc_link_while_moving_cancels_and_stops():
    e = eng()
    st, t = _moving(e)
    acts, t = run(e, t, st, link=False, armed=True)
    assert e.status == mc.CANCELLED and "liaison" in e.reason
    assert any(a["args"].get("command") == "cancel_all" for a in acts)


def test_disarming_while_moving_cancels_and_stops():
    e = eng()
    st, t = _moving(e)
    acts, t = run(e, t, st, link=True, armed=False)
    assert e.status == mc.CANCELLED and "desarm" in e.reason
    assert any(a["args"].get("command") == "cancel_all" for a in acts)


def test_cancel_returns_the_stop_and_does_not_shut_nav_down():
    e = eng()
    st, t = _moving(e)
    acts = e.cancel(t, "arret demande")
    calls = [(a["call"], a["args"]) for a in acts]
    assert ("/cmd", {"command": "cancel_all"}) in calls
    assert not any(c[0] == "/nav_stack" for c in calls)    # la pile NAV reste allumee


def test_step_timeout_fails_with_the_missing_condition():
    e = eng()
    st = {"noeuds": []}
    e.start("Prepa", {}, 0.0, st, True, False)
    run(e, 0.0, st, steps=100)
    assert e.status == mc.FAILED and "delai de 90 s" in e.reason and "noeud /nav_supervisor absent" in e.reason


def test_a_refused_call_fails_the_mission():
    e = eng()
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "idle"}
    e.start("Prepa", {}, 0.0, st, True, False)
    run(e, 0.0, st, steps=4)
    e.call_result(5.0, False, "refus du superviseur")
    assert e.status == mc.FAILED and "refus du superviseur" in e.reason


def test_a_failure_while_moving_sends_the_stop():
    e = eng()
    st, t = _moving(e)
    acts, t = run(e, t, st, steps=400, armed=True)         # le but n'aboutit jamais
    assert e.status == mc.FAILED and "delai" in e.reason
    assert any(a["args"].get("command") == "cancel_all" for a in acts)


def test_idle_engine_ticks_do_nothing():
    e = eng()
    assert e.tick(1.0, ST0, True, True) == [] and e.cancel(1.0) == []


def test_snapshot_reports_each_step_state():
    e = eng()
    st = {"noeuds": []}
    e.start("Prepa", {}, 0.0, st, True, False)
    run(e, 0.0, st, steps=2)
    s = e.snapshot(2.0)
    assert s["etat"] == "running" and s["etapes"][0]["etat"] in ("starting", "waiting_ready", "running")
    assert [x["id"] for x in s["etapes"]] == ["pile", "mode_nav", "reloc"]


# --- corrections du 2026-11-10 (analyse de la premiere mission reelle) -------------------------------------

def _hold_model(hold=15, garde=None, surveille=None):
    m = copy.deepcopy(MODEL)
    m["capacites"]["reloc"]["maintien_s"] = hold
    if garde:
        m["capacites"]["goto"]["garde"] = garde
    if surveille:
        m["capacites"]["goto"]["surveille"] = surveille
    return mc.MissionEngine(m)


def test_a_condition_must_hold_before_the_step_is_finished():
    """Releve reel : loc_state est passe a « ok » 26 s AVANT que la carte soit chargee ; 5 min plus tard 0,16."""
    e = _hold_model(hold=15)
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "inconnue"}
    e.start("Prepa", {}, 0.0, st, True, False)
    t = 0.0
    for _ in range(8):
        t += 1.0
        e.tick(t, st, True, False)
    st["nav.loc_state"] = "ok"
    for _ in range(10):                                    # « ok » pendant 10 s seulement...
        t += 1.0
        e.tick(t, st, True, False)
    assert e.status == mc.RUNNING and e.current() == "reloc"
    st["nav.loc_state"] = "douteuse"                      # ...puis elle retombe : le compteur repart de zero
    t += 1.0
    e.tick(t, st, True, False)
    st["nav.loc_state"] = "ok"
    for _ in range(10):
        t += 1.0
        e.tick(t, st, True, False)
    assert e.status == mc.RUNNING                          # 10 s depuis le retour a « ok » : pas assez
    for _ in range(8):
        t += 1.0
        e.tick(t, st, True, False)
    assert e.status == mc.DONE


def test_a_moving_step_is_refused_at_launch_when_the_localisation_fell_after_the_relocalisation():
    """La relocalisation finit « ok » (tenu 0 s ici), puis la localisation retombe AVANT le lancement du goto : la
    garde du lancement doit annuler, au lieu de partir sur une pose douteuse."""
    e = _hold_model(hold=0, garde={"nav.loc_state": ["ok"]})
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "ok"}
    e.start("Aller", {"destination": "p1"}, 0.0, st, True, True)
    t = 0.0
    while e.current() != "goto" and e.status == mc.RUNNING and t < 100:
        t += 1.0
        e.tick(t, st, True, True)
    assert e.current() == "goto" and e.step_state == mc.PENDING        # prete a partir, pas encore lancee
    st["nav.loc_state"] = "perdue"                                     # la localisation retombe juste avant
    t += 1.0
    acts = e.tick(t, st, True, True)
    assert e.status == mc.CANCELLED and "refusee" in e.reason and "nav.loc_state" in e.reason
    assert not any(a["args"].get("command") == "goto" for a in acts)  # jamais lance


def test_losing_the_localisation_while_moving_cancels_and_stops():
    e = _hold_model(garde={"nav.loc_state": ["ok"]}, surveille={"nav.loc_state": ["ok", "douteuse"]})
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "ok", "nav.goal_status": "active"}
    e.start("Aller", {"destination": "p1"}, 0.0, st, True, True)
    t = 0.0
    for _ in range(40):
        t += 1.0
        e.tick(t, st, True, True)
    assert e.current() == "goto" and e.step_state == mc.RUNNING_STEP
    st["nav.loc_state"] = "douteuse"                      # toleree : le robot continue
    t += 1.0
    assert e.tick(t, st, True, True) == [] and e.status == mc.RUNNING
    st["nav.loc_state"] = "perdue"                        # perdue : arret
    t += 1.0
    acts = e.tick(t, st, True, True)
    assert e.status == mc.CANCELLED and "nav.loc_state" in e.reason
    assert any(a["args"].get("command") == "cancel_all" for a in acts)


def test_the_duration_is_frozen_when_the_mission_ends():
    """Releve reel : duree_s 640,9 s pour une mission de 116,4 s."""
    e = eng()
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "ok"}
    e.start("Prepa", {}, 0.0, st, True, False)
    t = 0.0
    while e.status == mc.RUNNING and t < 100:
        t += 1.0
        e.tick(t, st, True, False)
    assert e.status == mc.DONE
    d1 = e.snapshot(t)["duree_s"]
    assert e.snapshot(t + 500.0)["duree_s"] == d1


def test_the_shipped_model_guards_every_moving_step():
    import yaml
    p = os.path.join(os.path.dirname(__file__), "..", "config", "missions.yaml")
    model = yaml.safe_load(open(p, encoding="utf-8"))
    for n, c in model["capacites"].items():
        if c.get("arme"):
            assert c.get("garde") and c.get("surveille"), "%s roule sans garde ni surveillance" % n
    assert model["capacites"]["relocalisation"]["maintien_s"] >= 10
