"""Nomenclature des parts (parts.yaml) : coherence interne, accord avec l'URDF BambooWS, et analyse d'incident (racine, impact) sur des pannes REELLES.

    cd bamboo_diagnostics && python3 -m pytest test/test_parts.py -q
"""
import copy
import os
import re
import sys

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))

from bamboo_diagnostics import parts_model as pm  # noqa: E402

REPO = os.path.join(HERE, "..", "..")
PARTS = os.path.join(REPO, "bamboo4WD_V4_WSEsp32_base", "config", "parts.yaml")
URDF = os.path.join(REPO, "linorobot2_description", "urdf", "robots", "bamboo_ws.urdf.xacro")


def _urdf_names():
    """Noms de liens et d'articulations ECRITS dans le xacro (les noms de roue sont fabriques par la macro : <cote>_wheel_link / _joint)."""
    text = open(URDF, encoding="utf-8").read()
    names = set(re.findall(r'<(?:link|joint) name="([a-z_0-9]+)"', text))
    for side in re.findall(r'<xacro:wheel side="([a-z_]+)"', text):
        names |= {side + "_wheel_link", side + "_wheel_joint"}
    return names


@pytest.fixture(scope="module")
def doc():
    return pm.load(PARTS)


@pytest.fixture(scope="module")
def graph(doc):
    return pm.Graph(doc)


def test_the_shipped_nomenclature_is_sound(doc):
    errs = pm.validate(doc, urdf_names=_urdf_names())
    assert errs == [], "\n".join(errs)


def test_every_urdf_link_has_a_part_or_is_a_known_frame(doc):
    """Aucune piece decrite dans l'URDF ne doit manquer a la nomenclature (les reperes purs sont listes ici)."""
    reperes = {"base_footprint"}
    ids = {p["id"] for p in doc["parts"]}
    # une roue est designee dans la nomenclature par son ARTICULATION (nom publie par le pilote dans /joint_states) : son lien en est la meme piece
    ids |= {i[:-len("_joint")] + "_link" for i in ids if i.endswith("_wheel_joint")}
    links = {n for n in _urdf_names() if not n.endswith("_joint") and "_to_" not in n}
    manque = sorted(links - reperes - ids)
    assert manque == [], "liens URDF sans part : %s" % manque


def test_wheel_joints_are_named_as_the_driver_publishes_them(doc):
    """Le pilote ESP32 publie /joint_states avec ces noms, dans l'ordre M1..M4 : la nomenclature doit les porter tels quels."""
    src = open(os.path.join(REPO, "bamboo_base", "bamboo_base", "esp32_mavlink_driver.py"), encoding="utf-8").read()
    names = re.findall(r'"([a-z_]+_wheel_joint)"', src[src.index("JOINT_NAMES"):src.index("JOINT_NAMES") + 300])
    assert len(names) == 4
    ids = [p["id"] for p in doc["parts"]]
    for n in names:
        assert n in ids, n


def test_the_urdf_has_no_simulation_residue():
    """On cherche dans le XML REEL : l'en-tete cite ces mots dans un commentaire pour expliquer ce qui a ete retire."""
    text = re.sub(r"<!--.*?-->", "", open(URDF, encoding="utf-8").read(), flags=re.S)
    for bad in ("<gazebo", "libgazebo", "camera_depth", "linorobot2_2wd", "hokuyo"):
        assert bad not in text, bad


def test_the_urdf_keeps_every_name_other_nodes_rely_on():
    names = _urdf_names()
    for n in ("base_footprint", "base_link", "imu_link", "laser", "camera_link", "front_left_wheel_joint", "front_right_wheel_joint",
              "rear_left_wheel_joint", "rear_right_wheel_joint"):
        assert n in names, n


def test_a_reference_is_never_invented(doc):
    """Une part « a_renseigner » n'a pas de reference ; une part « confirme » dit d'ou vient l'information."""
    for p in doc["parts"]:
        if p["certitude"] == "a_renseigner":
            assert not p.get("ref"), p["id"]
        if p["certitude"] == "confirme":
            assert p.get("source"), p["id"]


# ---------------------------------------------------------------------------- les defauts sont bien detectes
def _bad(doc, edit):
    d = copy.deepcopy(doc)
    edit(d)
    return pm.validate(d)


def test_validate_reports_every_defect(doc):
    def edit(d):
        d["parts"][1]["parent"] = "inconnu"
        d["parts"][2]["depends_on"] = ["nulle_part"]
        d["parts"][3]["type"] = "magique"
        d["parts"].append(dict(d["parts"][4]))                                  # id en double
    errs = _bad(doc, edit)
    texte = "\n".join(errs)
    for attendu in ("parent inconnu", "depends_on inconnu", "type 'magique'", "id en double"):
        assert attendu in texte, attendu


def test_a_dependency_cycle_is_refused(doc):
    def edit(d):
        by = {p["id"]: p for p in d["parts"]}
        by["batterie"]["depends_on"] = ["esp32_board"]                         # esp32_board depend deja de batterie
    assert any("cycle de depends_on" in e for e in _bad(doc, edit))


def test_a_parent_cycle_is_refused(doc):
    def edit(d):
        by = {p["id"]: p for p in d["parts"]}
        by["robot"]["parent"] = "esp32_board"
    errs = _bad(doc, edit)
    assert any("cycle de parent" in e for e in errs) or any("une racine" in e for e in errs)


def test_two_roots_are_refused(doc):
    def edit(d):
        d["parts"][2].pop("parent")
    assert any("UNE racine" in e for e in _bad(doc, edit))


def test_unknown_urdf_link_and_unknown_diag_task_are_refused(doc):
    errs = pm.validate(doc, urdf_names={"base_link"}, diag_names={"seule_tache"})
    texte = "\n".join(errs)
    assert "absent de l'URDF" in texte and "tache de diagnostic inconnue" in texte


# ---------------------------------------------------------------------------- analyse d'incident sur des pannes reelles
def test_the_frozen_webcam_of_2026_11_14_has_one_root_cause(graph):
    """Le 2026-11-14 la webcam UVC a gele : le diagnostic criait sur 4 services, la cause etait UNE seule piece."""
    en_defaut = ["camera_link", "bamboo_video", "camera_h264", "bamboo_videotracking"]
    assert graph.racines(en_defaut) == ["camera_link"]
    inc = graph.incidents(en_defaut)
    assert len(inc) == 1
    racine, consequences, impact = inc[0]
    assert racine == "camera_link"
    assert set(consequences) == {"bamboo_video", "camera_h264", "bamboo_videotracking"}


def test_an_esp32_link_loss_explains_the_driver_and_everything_that_rides_on_it(graph):
    en_defaut = ["cp2102_esp32", "driver_esp32", "bamboo_control", "navigation"]
    assert graph.racines(en_defaut) == ["cp2102_esp32"]
    assert "navigation" in graph.impact("cp2102_esp32")


def test_a_shared_usb_hub_is_the_root_when_esp32_lidar_and_camera_vanish_together(graph):
    """Lidar, ESP32 et webcam partagent le hub USB : si les trois tombent ensemble, la racine est le hub, pas trois pannes."""
    en_defaut = ["cp2102_esp32", "cp2102_lidar", "camera_link", "driver_esp32", "laser", "bamboo_video"]
    assert graph.racines(en_defaut) == ["hub_usb"] or graph.racines(en_defaut + ["hub_usb"]) == ["hub_usb"]


def test_a_battery_fault_is_the_root_of_a_whole_chain(graph):
    en_defaut = ["batterie", "esp32_board", "tb6612fng", "driver_esp32", "motor_fl", "front_left_wheel_joint"]
    assert graph.racines(en_defaut) == ["batterie"]


def test_independent_faults_stay_separate_and_the_critical_one_comes_first(graph):
    inc = graph.incidents(["manette", "cp2102_esp32", "driver_esp32"])
    assert [i[0] for i in inc] == ["cp2102_esp32", "manette"]                  # criticite 1 avant 2
    assert "driver_esp32" in inc[0][1]


def test_a_stopped_on_demand_service_alone_is_a_leaf_root(graph):
    """Le suivi arrete volontairement ne tire personne : aucun autre composant n'en depend."""
    assert graph.impact("bamboo_videotracking") == []


def test_causes_and_impact_are_mirror_images(graph, doc):
    for p in doc["parts"]:
        for c in graph.causes(p["id"]):
            assert p["id"] in graph.impact(c)


def test_every_diagnostic_task_named_in_the_nomenclature_follows_the_published_naming():
    """Toute tache citee suit la forme « noeud: nom » publiee par les diagnostics."""
    doc = pm.load(PARTS)
    forme = re.compile(r"^[a-z0-9_.]+: [a-zA-Z0-9_ .\-/]+$")
    for p in doc["parts"]:
        for t in p.get("diag") or []:
            assert forme.match(t), "%s : %r" % (p["id"], t)


def test_the_unplugged_lidar_of_2026_11_18_has_one_root_cause(graph):
    """INC-2026-11-18 : lidar debranche. Deux erreurs de diagnostic (scan_monitor, nav_supervisor), UNE seule racine : l'adaptateur du lidar."""
    en_defaut = ["cp2102_lidar", "laser", "navigation"]
    assert graph.racines(en_defaut) == ["cp2102_lidar"]
    racine, consequences, impact = graph.incidents(en_defaut)[0]
    assert racine == "cp2102_lidar" and set(consequences) == {"laser", "navigation"}


def test_the_lidar_incident_left_the_esp32_untouched(graph):
    """Le lidar et l'ESP32 partagent le hub USB mais le retrait du premier n'est PAS une panne du second : aucune dependance de l'un vers l'autre."""
    assert "driver_esp32" not in graph.impact("cp2102_lidar")
    assert "cp2102_esp32" not in graph.impact("cp2102_lidar")
    assert "cp2102_lidar" not in graph.causes("driver_esp32")


def test_the_lidar_resolution_says_the_driver_does_not_restart_by_itself(doc):
    """La piste de resolution doit contenir le piege decouvert : pas de reprise automatique apres un rebranchement."""
    texte = " ".join(p for part in doc["parts"] if part["id"] in ("laser", "cp2102_lidar") for p in part.get("resolution", []))
    assert "NE REPREND PAS SEUL" in texte or "ne reprend pas seul" in texte
    assert "restart driver.lidarD500" in texte


def test_the_lidar_adapter_is_watched_by_the_scan_monitor(doc):
    by = {p["id"]: p for p in doc["parts"]}
    assert "scan_monitor: lidar" in by["cp2102_lidar"]["diag"] and "scan_monitor: lidar" in by["laser"]["diag"]


def test_the_manual_proposes_improvements_and_marks_each_with_a_status():
    """Le manuel porte une section « Améliorations proposées » ; chaque amélioration a un statut connu et un motif (un incident réel)."""
    text = open(os.path.join(REPO, "docs", "depannage_bambooWS.md"), encoding="utf-8").read().replace("\r\n", "\n")
    sect = text[text.index("## 5. Améliorations proposées"):text.index("## 6. Comment ajouter un incident")]
    items = re.split(r"\n### ", sect)[1:]
    assert len(items) >= 5
    for it in items:
        assert re.search(r"\*\*Statut\*\* \| (proposée|validée|faite|rejetée)", it), it.splitlines()[0]
        assert "**Motif**" in it, it.splitlines()[0]


def test_the_manual_does_not_claim_that_a_docker_restart_alone_fixes_the_lidar():
    """Erreur evitee : `restart: on-failure` seul ne relance rien, scan_monitor garde le conteneur en vie. Le manuel doit le dire et proposer le respawn du noeud."""
    text = open(os.path.join(REPO, "docs", "depannage_bambooWS.md"), encoding="utf-8").read().replace("\r\n", "\n")
    assert "scan_monitor" in text and "respawn" in text
    assert "ne suffirait pas" in text or "ne répare pas seule" in text
