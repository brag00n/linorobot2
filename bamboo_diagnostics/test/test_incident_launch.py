"""diagnostics.launch.py : le noeud d'incident est lance par `python3 -m`, avec son fichier parts.yaml, et seulement si ce fichier existe.

Pourquoi `python3 -m` et pas `Node(executable=...)` (2026-11-18) : l'image `diagnostics` est dediee et minimale, `colcon build` y echoue (« Could not find a primary shell
extension ») et la reconstruire coute des heures. Le point d'entree declare dans setup.py n'y est jamais genere ; le MODULE, lui, est importable (installation en egg-link).

Le lanceur exige ROS : ce test se SAUTE ailleurs.
    docker exec bamboov4_humble-bamboo_diagnostics-1 bash -lc 'source /opt/ros/humble/setup.bash; \
        cd /root/linorobot2_ws/src/linorobot2/bamboo_diagnostics && python3 -m pytest test/test_incident_launch.py -q -p no:cacheprovider'
"""
import importlib.util
import os

import pytest

pytest.importorskip("launch")
pytest.importorskip("launch_ros")

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, "..", "launch", "diagnostics.launch.py")


def _setup(robot):
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location("diagnostics_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations["robot"] = robot
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("bamboo_diagnostics")
    except Exception:
        pytest.skip("bamboo_diagnostics non installe (pas d'espace de travail ROS)")
    return mod._launchSetup(ctx)


def _kinds(actions):
    return [type(a).__name__ for a in actions]


def _txt(x):
    """Une commande ou un message de lancement est une LISTE DE SUBSTITUTIONS, pas du texte : on les developpe (un TextSubstitution porte .text)."""
    if isinstance(x, (list, tuple)):
        return "".join(_txt(i) for i in x)
    return getattr(x, "text", str(x))


def test_bambooWS_gets_the_incident_node_launched_as_a_python_module():
    actions = _setup("bamboo4WD_V4_WSEsp32")
    execs = [a for a in actions if type(a).__name__ == "ExecuteProcess"]
    assert len(execs) == 1, _kinds(actions)
    cmd = [_txt(c) for c in execs[0].cmd]
    flat = " ".join(cmd)
    assert cmd[:3] == ["python3", "-m", "bamboo_diagnostics.incident_node"], cmd
    assert "__node:=incident_node" in flat
    assert "parts_file:=" in flat and flat.rstrip().endswith("parts.yaml")


def test_the_incident_node_is_not_a_ros_entry_point_here():
    """Le point d'entree n'existe pas dans cette image : aucun `Node(executable='incident_node')` ne doit rester dans le lanceur."""
    text = open(LAUNCH, encoding="utf-8").read().replace("\r\n", "\n")
    assert 'executable="incident_node"' not in text and "executable='incident_node'" not in text


def test_a_robot_without_parts_yaml_has_no_incident_node_and_says_so():
    """YBStm32 n'a pas (encore) de parts.yaml : pas de noeud, et un message dans le journal du lanceur, pas une erreur."""
    actions = _setup("bamboo4WD_V4_YBStm32")
    assert not [a for a in actions if type(a).__name__ == "ExecuteProcess"]
    textes = " ".join(_txt(getattr(a, "msg", "")) for a in actions if type(a).__name__ == "LogInfo")
    assert "pas de parts.yaml" in textes or "pas d'analyse d'incident" in textes


def test_the_two_historical_nodes_are_still_launched():
    actions = _setup("bamboo4WD_V4_WSEsp32")
    noms = {getattr(a, "node_executable", None) for a in actions}
    assert {"aggregator_node", "health_node"} <= noms
