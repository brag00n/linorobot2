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


def test_losing_the_cc_link_does_not_cancel_the_mission():
    """Le moteur ne juge plus la liaison : c'est le garde-fou qui desarme (armement CC simple). La mission n'a pas a l'annuler."""
    e = eng()
    st, t = _moving(e)
    acts, t = run(e, t, st, link=False, armed=True)
    assert e.status == mc.RUNNING and e.step_state == mc.RUNNING_STEP
    assert not any(a.get("args", {}).get("command") == "cancel_all" for a in acts)


def test_disarming_while_moving_pauses_the_step_instead_of_cancelling():
    e = eng()
    st, t = _moving(e)
    acts, t = run(e, t, st, link=True, armed=False)
    assert e.status == mc.RUNNING and e.step_state == mc.PAUSED and e.current() == "goto"
    assert e.snapshot(t)["pause"] is True
    assert not any(a.get("args", {}).get("command") == "cancel_all" for a in acts)   # le teleop a deja arrete le robot


def test_rearming_resumes_the_step_where_it_was():
    e = eng()
    st, t = _moving(e)
    run(e, t, st, link=True, armed=False)
    assert e.step_state == mc.PAUSED
    acts, t = run(e, t + 1, st, link=True, armed=True, steps=6)
    assert e.status == mc.RUNNING and e.current() == "goto"
    assert any(a.get("args", {}).get("command") == "goto" for a in acts)               # l'action de l'etape est RELANCEE
    assert e.snapshot(t)["pause"] is False


def test_the_pause_does_not_consume_the_step_timeout():
    e = eng()
    st, t = _moving(e)
    run(e, t, st, link=True, armed=False)
    t += 400.0                                                                         # plus que delai_s de goto (300)
    e.tick(t, st, True, False)
    assert e.status == mc.RUNNING and e.step_state == mc.PAUSED
    acts, t = run(e, t, st, link=True, armed=True, steps=4)
    assert e.status == mc.RUNNING                                                      # le delai repart a la reprise


def test_a_pause_longer_than_ten_minutes_cancels_the_mission():
    e = eng()
    st, t = _moving(e)
    run(e, t, st, link=True, armed=False)
    assert e.step_state == mc.PAUSED
    t += mc.PAUSE_MAX_S + 5
    acts = e.tick(t, st, True, False)
    assert e.status == mc.CANCELLED and "pause" in e.reason
    assert any(a.get("args", {}).get("command") == "cancel_all" for a in acts)


def test_resuming_needs_nothing_but_the_armed_traction():
    """Decision de l'utilisateur : pas de test supplementaire (liaison, localisation) a la reprise."""
    e = eng()
    st, t = _moving(e)
    run(e, t, st, link=True, armed=False)
    st2 = dict(st, **{"nav.loc_state": "douteuse"})
    acts, t = run(e, t + 1, st2, link=False, armed=True, steps=4)
    assert e.status == mc.RUNNING and e.step_state != mc.PAUSED


def test_a_moving_step_waits_instead_of_cancelling_when_the_traction_is_not_armed_yet():
    e = _hold_model(hold=0)
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "ok", "nav.goal_status": ""}
    e.start("Aller", {"destination": "p1"}, 0.0, st, True, True)
    t = 0.0
    while e.current() != "goto" and t < 100:
        t += 1.0
        e.tick(t, st, True, True)
    t += 1.0
    e.tick(t, st, True, False)                                                         # le robot est desarme juste avant l'etape
    assert e.status == mc.RUNNING and e.step_state == mc.PAUSED
    t += 1.0
    acts = e.tick(t, st, True, True)                                                   # puis re-arme : l'etape part
    assert e.status == mc.RUNNING and e.step_state != mc.PAUSED


def test_cancel_during_a_pause_still_works():
    e = eng()
    st, t = _moving(e)
    run(e, t, st, link=True, armed=False)
    acts = e.cancel(t + 3, "annulation demandee")
    assert e.status == mc.CANCELLED and any(a.get("args", {}).get("command") == "cancel_all" for a in acts)


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


# --- services d'une mission (2026-11-11) : eteindre le superflu AVANT de demarrer le necessaire, remettre a la fin ---------------

SVC_MODEL = {
    "socle": ["driver.esp32", "bamboo_control", "rosbridge"],
    "optionnels": ["bamboo_videotracking", "bamboo_calibration", "driver.lidarD500", "navigation"],
    "capacites": {
        "pile": {"services": ["driver.lidarD500", "navigation"], "demarre": CALL("/nav_stack", data=True),
                 "pret_quand": {"noeud": "/nav_supervisor"}, "delai_s": 90},
        "reloc": {"exige": ["pile"], "lance": CALL("/cmd", command="relocalize"), "fini_quand": {"nav.loc_state": ["ok"]}, "delai_s": 120},
        "goto": {"exige": ["reloc"], "arme": True, "lance": CALL("/cmd", command="goto"), "fini_quand": {"nav.goal_status": ["succeeded"]},
                 "delai_s": 300, "annule": CALL("/cmd", command="cancel_all")},
    },
    "missions": {"Prepa": {"etapes": ["reloc"]}, "Aller": {"etapes": ["goto"]}},
}
RUN_ALL = ["driver.esp32", "bamboo_control", "rosbridge", "bamboo_videotracking", "bamboo_calibration"]


def test_services_needed_is_the_union_of_the_capabilities():
    assert mc.services_needed(SVC_MODEL, "Prepa") == ["driver.lidarD500", "navigation"]
    assert mc.services_needed(SVC_MODEL, "Aller") == ["driver.lidarD500", "navigation"]        # goto n'en ajoute pas


def test_the_plan_stops_the_useless_optional_services_and_starts_the_missing_ones():
    p = mc.service_plan(SVC_MODEL, "Prepa", RUN_ALL)
    assert p["stop"] == ["bamboo_videotracking", "bamboo_calibration"]
    assert p["start"] == ["driver.lidarD500", "navigation"]
    assert p["restore"] == ["bamboo_videotracking", "bamboo_calibration"]


def test_the_socle_is_never_stopped():
    p = mc.service_plan(SVC_MODEL, "Prepa", RUN_ALL)
    assert not set(p["stop"]) & {"driver.esp32", "bamboo_control", "rosbridge"}


def test_a_service_outside_the_two_lists_is_ignored():
    p = mc.service_plan(SVC_MODEL, "Prepa", RUN_ALL + ["portainer", "inconnu"])
    assert "portainer" not in p["stop"] and "inconnu" not in p["stop"]


def test_nothing_is_stopped_that_is_already_off_and_nothing_started_that_runs():
    p = mc.service_plan(SVC_MODEL, "Prepa", ["driver.esp32", "driver.lidarD500", "navigation"])
    assert p == {"stop": [], "start": [], "restore": []}


def test_start_emits_stop_before_start():
    """L'ordre de l'utilisateur : eteindre d'abord, pour ne pas ajouter une surcharge CPU au demarrage."""
    e = mc.MissionEngine(copy.deepcopy(SVC_MODEL))
    ok, _why, acts = e.start("Prepa", {}, 0.0, {"services": RUN_ALL}, True, False)
    assert ok and [a["services"] for a in acts] == ["stop", "start"]
    assert acts[0]["names"] == ["bamboo_videotracking", "bamboo_calibration"]
    assert acts[1]["names"] == ["driver.lidarD500", "navigation"]


def _finish(e, st, armed=False, steps=400):
    t, acts = 0.0, []
    for _ in range(steps):
        t += 1.0
        acts += e.tick(t, st, True, armed)
        if e.status != mc.RUNNING:
            break
    return acts


def test_services_are_restored_when_the_mission_is_done():
    e = mc.MissionEngine(copy.deepcopy(SVC_MODEL))
    st = {"services": RUN_ALL, "noeuds": ["/nav_supervisor"], "nav.loc_state": "ok"}
    e.start("Prepa", {}, 0.0, st, True, False)
    acts = _finish(e, st)
    assert e.status == mc.DONE
    restore = [a for a in acts if a.get("services") == "start"]
    assert restore and restore[-1]["names"] == ["bamboo_videotracking", "bamboo_calibration"]


def test_services_are_restored_after_a_failure():
    e = mc.MissionEngine(copy.deepcopy(SVC_MODEL))
    st = {"services": RUN_ALL, "noeuds": []}                         # la pile ne devient jamais prete : delai
    e.start("Prepa", {}, 0.0, st, True, False)
    acts = _finish(e, st)
    assert e.status == mc.FAILED
    assert any(a.get("services") == "start" and "bamboo_videotracking" in a["names"] for a in acts)


def test_services_are_restored_after_a_cancel_and_the_stop_goes_first():
    e = mc.MissionEngine(copy.deepcopy(SVC_MODEL))
    st = {"services": RUN_ALL, "noeuds": ["/nav_supervisor"], "nav.loc_state": "ok", "nav.goal_status": "active"}
    e.start("Aller", {}, 0.0, st, True, True)
    t = 0.0
    for _ in range(40):
        t += 1.0
        e.tick(t, st, True, True)
    acts = e.cancel(t, "arret demande")
    kinds = [("stop-robot" if a.get("call") else a["services"]) for a in acts]
    assert kinds[0] == "stop-robot" and kinds[-1] == "start"       # cancel_all d'abord, puis les services remis
    assert e.restore_actions() == []                               # une seule fois


def test_nothing_to_restore_when_nothing_was_stopped():
    e = mc.MissionEngine(copy.deepcopy(SVC_MODEL))
    e.start("Prepa", {}, 0.0, {"services": ["driver.esp32"]}, True, False)
    assert e.restore_actions() == []


def test_validate_rejects_an_unknown_service():
    bad = copy.deepcopy(SVC_MODEL)
    bad["capacites"]["pile"]["services"] = ["service_fantome"]
    with pytest.raises(mc.MissionError, match="service_fantome"):
        mc.validate(bad)


def test_validate_rejects_a_service_in_both_lists():
    bad = copy.deepcopy(SVC_MODEL)
    bad["optionnels"].append("rosbridge")
    with pytest.raises(mc.MissionError, match="socle et optionnel"):
        mc.validate(bad)


def test_the_shipped_model_keeps_the_pilot_services_in_the_socle():
    import yaml
    model = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "missions.yaml"), encoding="utf-8"))
    mc.validate(model)
    for needed in ("driver.esp32", "bamboo_control", "rosbridge", "camera.h264", "bamboo_video", "battery_energy"):
        assert needed in model["socle"], needed
    assert "bamboo_videotracking" in model["optionnels"]
    assert not set(model["socle"]) & set(model["optionnels"])


# --- mission « Exploration carte vierge » (2026-11-11) -------------------------------------------------------------

def _shipped():
    import yaml
    return yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "missions.yaml"), encoding="utf-8"))


def test_the_blank_map_mission_skips_the_relocalisation():
    """Sur une carte neuve il n'y a rien a relocaliser : le robot part de l'origine (le superviseur pose loc = ok)."""
    m = _shipped()
    plan = mc.compose(m, "Exploration_carte_vierge")
    assert plan == ["pile_nav", "mode_robot_nav", "mode_exploration_vierge", "exploration_vierge"]
    assert "relocalisation" not in plan
    assert "relocalisation" in mc.compose(m, "Exploration")                        # l'exploration ordinaire, elle, relocalise


def test_the_blank_map_mission_creates_a_new_map_and_still_guards_the_driving_step():
    m = _shipped()
    assert m["capacites"]["mode_exploration_vierge"]["lance"]["args"] == {"command": "exploration", "arg": "new"}
    assert m["capacites"]["mode_exploration"]["lance"]["args"]["arg"] == ""        # l'autre ne cree jamais de carte
    c = m["capacites"]["exploration_vierge"]
    assert c["arme"] is True and c["garde"] and c["surveille"] and c["annule"]


def test_the_blank_map_mission_needs_the_same_services_as_the_other_exploration():
    m = _shipped()
    assert mc.services_needed(m, "Exploration_carte_vierge") == mc.services_needed(m, "Exploration")


def test_the_blank_map_mission_is_refused_without_the_traction_armed():
    e = mc.MissionEngine(_shipped())
    ok, why, _ = e.start("Exploration_carte_vierge", {}, 0.0, {}, True, False)
    assert not ok and "armee" in why


# ---- debut_quand : un arret immediat n'est pas une fin (2026-11-11) ----
def _explore_model():
    m = copy.deepcopy(MODEL)
    m["capacites"]["explo"] = {"exige": ["pile", "mode_nav"], "arme": True,
                               "lance": CALL("/cmd", command="auto_explore"),
                               "debut_quand": {"nav.auto_state": ["exploration"]}, "debut_delai_s": 10,
                               "fini_quand": {"nav.auto_state": ["off"]}, "delai_s": 900}
    m["missions"]["Explo"] = {"etapes": ["explo"]}
    return mc.MissionEngine(m)


def test_an_exploration_that_stops_at_once_fails_instead_of_finishing():
    e = _explore_model()
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "ok", "nav.auto_state": "off",
          "nav.detail": "sortie du mode NAV"}
    e.start("Explo", {}, 0.0, st, True, True)
    run(e, 0.0, st, link=True, armed=True, steps=40)
    assert e.status == mc.FAILED and "sortie du mode NAV" in e.reason and "n'a pas demarre" in e.reason


def test_an_exploration_seen_running_then_off_finishes():
    e = _explore_model()
    st = {"noeuds": ["/nav_supervisor"], "nav.submode": "navigation", "nav.loc_state": "ok", "nav.auto_state": "off"}
    e.start("Explo", {}, 0.0, st, True, True)
    t = 0.0
    for _ in range(6):
        _a, t = run(e, t, st, armed=True)
    st["nav.auto_state"] = "exploration"
    _a, t = run(e, t, st, armed=True, steps=3)
    assert e.status == mc.RUNNING
    st["nav.auto_state"] = "off"
    run(e, t, st, armed=True, steps=3)
    assert e.status == mc.DONE


def test_the_shipped_exploration_missions_need_the_nav_robot_mode_and_a_start_condition():
    import yaml
    m = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "missions.yaml"), encoding="utf-8"))
    for mission in ("Exploration", "Exploration_carte_vierge", "Navigation", "Ronde", "Retour"):
        assert "mode_robot_nav" in mc.compose(m, mission)
    for cap in ("exploration", "exploration_vierge", "ronde"):
        assert m["capacites"][cap].get("debut_quand")
    assert m["capacites"]["mode_robot_nav"]["lance"]["service"] == "/bamboo/change_mode"


def test_a_text_condition_matches_a_substring_of_the_supervisor_message():
    cond = {"nav.detail~": ["slam_toolbox (mapping) actif"]}
    assert not mc.check(cond, {"nav.detail": "creation en cours"})
    assert not mc.check(cond, {})
    assert mc.check(cond, {"nav.detail": "nouvelle carte c : slam_toolbox (mapping) actif"})
    assert "nav.detail" in mc.why_not(cond, {"nav.detail": "x"})


def test_the_blank_map_step_waits_for_the_end_of_the_map_creation():
    import yaml
    m = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "missions.yaml"), encoding="utf-8"))
    assert "nav.detail~" in m["capacites"]["mode_exploration_vierge"]["fini_quand"]
    for cap in ("exploration", "exploration_vierge"):
        assert m["capacites"][cap]["debut_delai_s"] >= 45
