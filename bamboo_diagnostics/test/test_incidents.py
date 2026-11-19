"""incident_model : de l'arbre de diagnostic aux incidents. Pannes REELLES rejouees (webcam gelee du 2026-11-14, lidar debranche du 2026-11-18) et regles « pour ne pas mentir ».

    cd bamboo_diagnostics && python3 -m pytest test/test_incidents.py -q       (PyYAML : dans le conteneur, pas sur le PC)
"""
import os
import sys
from types import SimpleNamespace as NS

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))

from bamboo_diagnostics import incident_model as im  # noqa: E402
from bamboo_diagnostics import parts_model as pm  # noqa: E402

PARTS = os.path.join(HERE, "..", "..", "bamboo4WD_V4_WSEsp32_base", "config", "parts.yaml")
OK, WARN, ERROR, STALE = 0, 1, 2, 3

# Arbre de reference : toutes les taches de l'arbre en direct, au repos (suivi arrete, pile NAV arretee), toutes OK sauf ce qu'on y change.
TACHES = [
    "esp32_mavlink_driver: actuation", "esp32_mavlink_driver: batterie", "esp32_mavlink_driver: compensation", "esp32_mavlink_driver: configuration carte",
    "esp32_mavlink_driver: imu", "esp32_mavlink_driver: liaison serie", "bluetooth: adaptateur", "js0: peripherique", "joy_linux_node: presence",
    "bamboo_teleop: chemin servo", "bamboo_teleop: manette", "bamboo_teleop: modes", "bamboo_teleop: traction", "gamepad_state: sortie",
    "bamboo_mode_manager: modes du robot", "cmd_vel_mux: multiplexeur", "bamboo_health: observateur", "camera_fps: presence", "gscam_node: presence",
    "stream_mux_node: flux video", "web_video_server: cadence flux", "web_video_server: presence", "face_train_node: apprentissage", "facerecog_node: reconnaissance",
    "overlay_node: enregistrement mux", "overlay_node: incrustation", "tracking_node: detection", "mediamtx: flux H.264", "robot_state_publisher: presence",
    "rosbridge_websocket: presence", "scan_monitor: lidar", "nav_supervisor: navigation", "systeme: alimentation", "systeme: disque", "systeme: memoire",
    "systeme: occupation CPU", "systeme: temperature CPU",
]
SUIVI = ["face_train_node: apprentissage", "facerecog_node: reconnaissance", "overlay_node: enregistrement mux", "overlay_node: incrustation", "tracking_node: detection"]


def arbre(**niveaux):
    """Liste de DiagnosticStatus factices. niveaux : nom de tache (espaces -> _ non supportes : on passe un dict via **{...}) -> niveau."""
    out = []
    for t in TACHES:
        lvl = niveaux.get(t, STALE if t in SUIVI else OK)
        out.append(NS(name="/Robot/module (m)/composant/" + t, level=lvl, message="msg " + t, hardware_id=""))
        out.append(NS(name="/Robot/module (m)/composant", level=lvl, message="Warning", hardware_id=""))     # noeud intermediaire : ignore
    return out


@pytest.fixture(scope="module")
def doc():
    return pm.load(PARTS)


@pytest.fixture(scope="module")
def graph(doc):
    return pm.Graph(doc)


@pytest.fixture(scope="module")
def idx(doc):
    return im.index_diag(doc)


class Horloge:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def cycle(graph, idx, statuts, tracker, **hote):
    f = im.feuilles(statuts)
    bad, par, orph, ign = im.parts_en_defaut(f, idx, hote.get("etat"), hote.get("age"))
    actifs, resolus = tracker.update(bad, par, f)
    return actifs, resolus, orph, ign, f


# ------------------------------------------------------------------------------------------------ l'etat de repos n'est PAS un incident
def test_the_resting_robot_has_no_incident(graph, idx):
    """Suivi arrete (STALE x5), pile NAV arretee : c'est l'etat voulu, pas une panne."""
    t = im.IncidentTracker(graph, confirmations=1)
    actifs, _r, orph, ign, _f = cycle(graph, idx, arbre(), t)
    assert actifs == [] and orph == []
    assert set(ign) == set(SUIVI)                        # ignorees car attendues, et on le DIT


def test_a_stopped_tracking_service_is_not_reported_but_a_long_running_silent_one_is(graph, idx):
    t = im.IncidentTracker(graph, confirmations=1)
    actifs, *_ = cycle(graph, idx, arbre(), t, etat="running", age=20.0)        # lance depuis 20 s : on attend
    assert actifs == []
    t2 = im.IncidentTracker(graph, confirmations=1)
    actifs, *_ = cycle(graph, idx, arbre(), t2, etat="running", age=300.0)      # lance depuis 5 min, toujours muet : anomalie
    assert [a["racine"] for a in actifs] == ["bamboo_videotracking"]


def test_the_synthesis_task_is_never_analysed(graph, idx):
    """Boucle evitee : la synthese publiee dans /diagnostics revient dans /diagnostics_agg."""
    st = arbre() + [NS(name="/Robot/bamboo_diagnostics/" + im.SELF_TASK, level=ERROR, message="x", hardware_id="")]
    t = im.IncidentTracker(graph, confirmations=1)
    actifs, _r, orph, _i, _f = cycle(graph, idx, st, t)
    assert actifs == [] and im.SELF_TASK not in orph


# ------------------------------------------------------------------------------------------------ les pannes reelles
def test_the_unplugged_lidar_of_2026_11_18_is_one_incident(graph, idx):
    st = arbre(**{"scan_monitor: lidar": ERROR, "nav_supervisor: navigation": ERROR})
    t = im.IncidentTracker(graph, confirmations=1)
    actifs, _r, orph, _i, _f = cycle(graph, idx, st, t)
    assert orph == []
    assert len(actifs) == 1, [a["racine"] for a in actifs]
    inc = actifs[0]
    assert inc["racine"] in ("cp2102_lidar", "laser") and "navigation" in (inc["consequences"] + [inc["racine"]])
    assert any("driver.lidarD500" in r for r in inc["resolution"]) or inc["racine"] == "laser" and inc["resolution"]
    assert set(inc["taches"]) == {"scan_monitor: lidar", "nav_supervisor: navigation"}


def test_the_frozen_webcam_of_2026_11_14_is_one_incident_for_four_services(graph, idx):
    st = arbre(**{"stream_mux_node: flux video": ERROR, "web_video_server: cadence flux": ERROR, "overlay_node: incrustation": ERROR, "tracking_node: detection": ERROR,
                  "gscam_node: presence": OK})
    t = im.IncidentTracker(graph, confirmations=1)
    actifs, *_ = cycle(graph, idx, st, t, etat="running", age=300.0)
    racines = [a["racine"] for a in actifs]
    assert racines == ["camera_link"], racines
    assert "bamboo_video" in actifs[0]["consequences"]


def test_two_independent_faults_stay_two_incidents_most_critical_first(graph, idx):
    st = arbre(**{"js0: peripherique": WARN, "esp32_mavlink_driver: liaison serie": ERROR})
    t = im.IncidentTracker(graph, confirmations=1)
    actifs, *_ = cycle(graph, idx, st, t)
    racines = [a["racine"] for a in actifs]
    assert racines[0] == "cp2102_esp32" and "manette" in racines                  # criticite 1 avant 2
    assert actifs[0]["criticite"] <= actifs[1]["criticite"]


# ------------------------------------------------------------------------------------------------ regles « pour ne pas mentir »
def test_a_failing_task_with_no_part_is_reported_not_swallowed(graph, idx):
    st = arbre() + [NS(name="/Robot/x/y/inconnu_node: truc", level=ERROR, message="?", hardware_id="")]
    t = im.IncidentTracker(graph, confirmations=1)
    actifs, _r, orph, *_ = cycle(graph, idx, st, t)
    assert orph == ["inconnu_node: truc"]
    lvl, msg, vals = im.synthese(actifs, orph)
    assert lvl == im.WARN and "inconnu_node: truc" in msg


def test_a_fault_needs_two_consecutive_samples_before_it_becomes_an_incident(graph, idx):
    t = im.IncidentTracker(graph, confirmations=2)
    st = arbre(**{"esp32_mavlink_driver: batterie": ERROR})
    a1, *_ = cycle(graph, idx, st, t)
    assert a1 == []                                                               # un seul releve : pas encore
    a2, *_ = cycle(graph, idx, st, t)
    assert [a["racine"] for a in a2] == ["batterie"]


def test_a_one_sample_blip_never_becomes_an_incident(graph, idx):
    t = im.IncidentTracker(graph, confirmations=2)
    cycle(graph, idx, arbre(**{"esp32_mavlink_driver: batterie": ERROR}), t)
    cycle(graph, idx, arbre(), t)                                                 # retour a la normale
    a3, *_ = cycle(graph, idx, arbre(**{"esp32_mavlink_driver: batterie": ERROR}), t)
    assert a3 == []                                                               # le compteur est reparti de zero


def test_a_resolved_incident_stays_visible_then_disappears(graph, idx):
    h = Horloge()
    t = im.IncidentTracker(graph, confirmations=1, garde_resolu_s=30.0, clock=h)
    cycle(graph, idx, arbre(**{"js0: peripherique": WARN}), t)
    h.t += 5
    actifs, resolus, *_ = cycle(graph, idx, arbre(), t)
    assert actifs == [] and [r["racine"] for r in resolus] == ["manette"]
    h.t += 40
    actifs, resolus, *_ = cycle(graph, idx, arbre(), t)
    assert resolus == []


def test_an_incident_keeps_its_start_time_across_samples(graph, idx):
    h = Horloge()
    t = im.IncidentTracker(graph, confirmations=1, clock=h)
    st = arbre(**{"js0: peripherique": WARN})
    cycle(graph, idx, st, t)
    h.t += 12
    actifs, *_ = cycle(graph, idx, st, t)
    assert actifs[0]["depuis_s"] == pytest.approx(12.0)


# ------------------------------------------------------------------------------------------------ sorties
def test_the_payload_is_json_serialisable_and_has_no_internal_field(graph, idx):
    import json
    t = im.IncidentTracker(graph, confirmations=1)
    actifs, resolus, orph, ign, f = cycle(graph, idx, arbre(**{"scan_monitor: lidar": ERROR}), t)
    body = im.payload(actifs, resolus, orph, ign, len(f))
    s = json.dumps(body, ensure_ascii=False)
    assert "depuis\"" not in s and body["n_incidents"] == 1 and body["version"] == 1
    assert body["n_taches_analysees"] == len(TACHES)


def test_the_synthesis_levels(graph, idx):
    t = im.IncidentTracker(graph, confirmations=1)
    a, _r, o, *_ = cycle(graph, idx, arbre(), t)
    assert im.synthese(a, o)[0] == im.OK
    a, _r, o, *_ = cycle(graph, idx, arbre(**{"esp32_mavlink_driver: liaison serie": ERROR}), t)
    lvl, msg, vals = im.synthese(a, o)
    assert lvl == im.ERROR and "cp2102_esp32" in msg and ("racine_principale", "cp2102_esp32") in vals
    a, _r, o, *_ = cycle(graph, idx, arbre(**{"js0: peripherique": WARN}), t)
    assert im.synthese(a, o)[0] == im.WARN


def test_every_published_task_of_the_robot_is_attached_to_a_part(graph, idx):
    """Garde de couverture : aucune des taches de l'arbre en direct ne doit rester sans part (sinon elle serait « non rattachee » en permanence)."""
    manque = [t for t in TACHES if t not in idx]
    assert manque == [], manque


def test_intermediate_nodes_are_ignored(graph, idx):
    st = [NS(name="/Robot/bamboo_control (manette)", level=ERROR, message="Error", hardware_id="")]
    assert im.feuilles(st) == {}


def test_on_demand_parts_are_a_closed_known_list(doc):
    ids = {p["id"] for p in doc["parts"]}
    assert set(im.A_LA_DEMANDE) <= ids


def test_a_byte_level_from_rclpy_is_read_as_an_integer(graph, idx):
    """Defaut reel du 2026-11-18 : le champ `level` de DiagnosticStatus est un `byte`, il arrive en bytes (b'\x02'). Les statuts de test portaient des entiers."""
    assert im.niveau_de(b"\x00") == 0 and im.niveau_de(b"\x02") == 2 and im.niveau_de(b"\x03") == 3
    assert im.niveau_de(2) == 2 and im.niveau_de(bytearray(b"\x01")) == 1
    st = [NS(name="/Robot/m (x)/c/scan_monitor: lidar", level=b"\x02", message="aucun scan recu", hardware_id=""),
          NS(name="/Robot/m (x)/c/bamboo_health: observateur", level=b"\x00", message="en marche", hardware_id="")]
    f = im.feuilles(st)
    assert f["scan_monitor: lidar"][0] == 2 and f["bamboo_health: observateur"][0] == 0


def test_the_whole_chain_accepts_byte_levels(graph, idx):
    st = [NS(name="/Robot/m (x)/c/" + t, level=bytes([STALE if t in SUIVI else OK]), message="x", hardware_id="") for t in TACHES]
    st = [s if "scan_monitor" not in s.name else NS(name=s.name, level=b"\x02", message="aucun scan recu", hardware_id="") for s in st]
    t = im.IncidentTracker(graph, confirmations=1)
    actifs, *_ = cycle(graph, idx, st, t)
    assert [a["racine"] for a in actifs] in (["laser"], ["cp2102_lidar"])


def test_a_battery_warning_does_not_swallow_an_unplugged_lidar_and_a_switched_off_pad(graph, idx):
    """Constate le 2026-11-19 : un simple avertissement de batterie devenait la « cause » du lidar debranche et de la manette eteinte."""
    f = im.feuilles(arbre(**{"esp32_mavlink_driver: batterie": WARN, "scan_monitor: lidar": ERROR, "js0: peripherique": WARN}))
    bad, par_part, _o, _i = im.parts_en_defaut(f, idx)
    actifs, _ = im.IncidentTracker(graph, confirmations=1).update(bad, par_part, f)
    racines = {a["racine"]: a for a in actifs}
    assert "batterie" in racines, list(racines)
    assert set(racines["batterie"]["consequences"]).isdisjoint({"cp2102_lidar", "laser"}), racines["batterie"]["consequences"]
    assert any(r in racines for r in ("cp2102_lidar", "laser")), list(racines)          # le lidar a son propre incident


def test_each_consequence_carries_its_own_label_level_and_tasks(graph, idx):
    f = im.feuilles(arbre(**{"esp32_mavlink_driver: liaison serie": ERROR, "esp32_mavlink_driver: batterie": ERROR, "esp32_mavlink_driver: imu": ERROR}))
    bad, par_part, _o, _i = im.parts_en_defaut(f, idx)
    actifs, _ = im.IncidentTracker(graph, confirmations=1).update(bad, par_part, f)
    assert any(a["consequences_detail"] for a in actifs), actifs
    for a in actifs:
        assert [c["id"] for c in a["consequences_detail"]] == a["consequences"]
        for c in a["consequences_detail"]:
            assert set(c) == {"id", "label", "niveau", "taches"} and c["niveau"] in ("WARN", "ERROR", "STALE") and c["label"]


# ------------------------------------------------------------------------------------------------ ordre : erreurs d'abord
def test_errors_come_before_warnings_whatever_the_criticality_of_the_root(graph, idx):
    """Un avertissement sur une part de criticite 1 (batterie) ne passe plus avant une erreur de criticite 2 (lidar)."""
    st = arbre(**{"esp32_mavlink_driver: batterie": WARN, "scan_monitor: lidar": ERROR, "js0: peripherique": WARN})
    actifs, *_ = cycle(graph, idx, st, im.IncidentTracker(graph, confirmations=1))
    niveaux = [a["niveau"] for a in actifs]
    assert niveaux == sorted(niveaux, key=lambda n: -im.RANG_NOM[n]), niveaux
    assert actifs[0]["niveau"] == "ERROR" and actifs[-1]["niveau"] == "WARN"


def test_among_equal_levels_the_incident_with_the_most_critical_piece_comes_first(graph, idx):
    st = arbre(**{"js0: peripherique": WARN, "esp32_mavlink_driver: liaison serie": WARN})                       # deux pannes INDEPENDANTES
    actifs, *_ = cycle(graph, idx, st, im.IncidentTracker(graph, confirmations=1))
    assert [a["racine"] for a in actifs][:2] == ["cp2102_esp32", "manette"], [a["racine"] for a in actifs]       # criticite 1 avant 2


def test_consequences_are_listed_most_serious_first(graph, idx):
    st = arbre(**{"esp32_mavlink_driver: liaison serie": ERROR, "esp32_mavlink_driver: imu": WARN, "esp32_mavlink_driver: batterie": ERROR})
    actifs, *_ = cycle(graph, idx, st, im.IncidentTracker(graph, confirmations=1))
    for a in actifs:
        niv = [c["niveau"] for c in a["consequences_detail"]]
        assert niv == sorted(niv, key=lambda n: -im.RANG_NOM[n]), (a["racine"], niv)
        assert [c["id"] for c in a["consequences_detail"]] == a["consequences"]


# ------------------------------------------------------------------------------------------------ surcharge CPU : durée minimale
CPU = "systeme: occupation CPU"


def test_a_cpu_overload_is_not_an_incident_before_it_lasts(graph, idx):
    h = Horloge()
    pers = im.Persistance(clock=h)
    st = arbre(**{CPU: ERROR})
    f, attente = pers.appliquer(im.feuilles(st))
    assert f[CPU][0] == OK and attente == [CPU]                                     # vue comme OK, mais signalee « en observation »
    h.t += 30
    assert pers.appliquer(im.feuilles(st))[0][CPU][0] == OK                         # 30 s : toujours un demarrage
    h.t += 31
    f, attente = pers.appliquer(im.feuilles(st))
    assert f[CPU][0] == ERROR and attente == []                                     # plus de 60 s sans interruption : c'est une vraie surcharge


def test_a_cpu_overload_that_stops_resets_the_counter(graph, idx):
    h = Horloge()
    pers = im.Persistance(clock=h)
    pers.appliquer(im.feuilles(arbre(**{CPU: WARN})))
    h.t += 50
    pers.appliquer(im.feuilles(arbre()))                                            # retour a OK
    h.t += 50
    f, attente = pers.appliquer(im.feuilles(arbre(**{CPU: WARN})))                  # rechute : le compteur repart de zero
    assert f[CPU][0] == OK and attente == [CPU]


def test_other_tasks_are_never_delayed(graph, idx):
    pers = im.Persistance(clock=Horloge())
    f, attente = pers.appliquer(im.feuilles(arbre(**{"scan_monitor: lidar": ERROR})))
    assert f["scan_monitor: lidar"][0] == ERROR and attente == []


def test_the_nav_stack_startup_spike_does_not_blame_the_rpi(graph, idx):
    """Constate le 2026-11-19 : la pile NAV demarree sans lidar faisait retenir le RPi comme racine de l'incident du lidar."""
    h = Horloge()
    pers = im.Persistance(clock=h)
    st = arbre(**{CPU: ERROR, "scan_monitor: lidar": ERROR, "nav_supervisor: navigation": ERROR})
    f, _a = pers.appliquer(im.feuilles(st))
    bad, par, _o, _i = im.parts_en_defaut(f, idx)
    actifs, _r = im.IncidentTracker(graph, confirmations=1).update(bad, par, f)
    assert "rpi" not in [a["racine"] for a in actifs], [a["racine"] for a in actifs]


def test_the_payload_lists_what_is_under_observation(graph, idx):
    assert im.payload([], [], [], [], 38, [CPU])["en_observation"] == [CPU]
    assert im.payload([], [], [], [], 38)["en_observation"] == []


# ------------------------------------------------------------------------------------------------ conseil adapte a ce qui tourne
NAV = ["nav_supervisor: navigation", "scan_monitor: lidar"]


def _conseils(graph, idx, **etat):
    st = arbre(**dict({CPU: ERROR}, **etat))
    actifs, *_ = cycle(graph, idx, st, im.IncidentTracker(graph, confirmations=1))
    return " ".join(next(a for a in actifs if a["racine"] == "rpi")["resolution"])


def test_the_advice_never_tells_to_stop_a_stack_that_is_not_running(graph, idx):
    texte = _conseils(graph, idx, **{t: STALE for t in NAV})                        # ni suivi ni NAV : leurs taches sont muettes (STALE), comme sur le robot au repos
    assert "couper le suivi" not in texte and "pile NAV (Nav2" not in texte
    assert "docker stats" in texte


def test_with_the_tracking_running_the_advice_is_to_stop_the_tracking(graph, idx):
    suivi = {t: OK for t in SUIVI}
    texte = _conseils(graph, idx, **suivi)
    assert "couper le suivi" in texte and "pile NAV (Nav2" not in texte


def test_with_only_the_nav_running_the_advice_is_to_stop_the_nav(graph, idx):
    texte = _conseils(graph, idx, **{"nav_supervisor: navigation": OK, "scan_monitor: lidar": OK})
    assert "pile NAV (Nav2" in texte and "couper le suivi" not in texte
