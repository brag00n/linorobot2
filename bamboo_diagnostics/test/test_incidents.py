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
