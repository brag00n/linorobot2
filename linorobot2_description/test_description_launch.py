"""description.launch.py : le parametre robot_description doit etre une CHAINE, quel que soit le contenu des commentaires XML de l'URDF.

Pourquoi ce test existe (2026-11-18) : le lanceur passait la sortie de xacro en `Command` nu, et launch_ros tente d'abord de l'interpreter comme du YAML. Un commentaire XML
contenant « : » (par exemple « [mesure] lidar.pose_x : 4 cm ») ressemble a `cle: valeur` : le lancement echouait en boucle (« Unable to parse the value of parameter
robot_description as yaml ») et robotdesc ne demarrait plus. 4wd.urdf.xacro y echappait par chance, bamboo_ws.urdf.xacro non.

Ce test exige ROS (launch_ros, xacro, un espace de travail construit) : il s'execute DANS le conteneur et SE SAUTE ailleurs.
    docker exec <conteneur> bash -lc 'source /opt/ros/humble/setup.bash; source /root/linorobot2_ws/install/setup.bash; \
        cd /root/linorobot2_ws/src/linorobot2 && python3 -m pytest linorobot2_description/test_description_launch.py -q'
"""
import os
import re
import subprocess
import sys

import pytest

HERE = os.path.dirname(__file__)
LAUNCH = os.path.join(HERE, "launch", "description.launch.py")
ROBOTS = os.path.join(HERE, "urdf", "robots")

launch = pytest.importorskip("launch")
pytest.importorskip("launch_ros")
yaml = pytest.importorskip("yaml")
HAS_XACRO = subprocess.run(["bash", "-lc", "command -v xacro"], capture_output=True).returncode == 0
needs_xacro = pytest.mark.skipif(not HAS_XACRO, reason="xacro absent")


def _load():
    """Le fichier s'appelle description.launch.py : le point du nom l'empeche d'etre importe comme module, on le charge par son chemin."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("description_launch", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _expanded_robot_description(urdf_file):
    """Compose le noeud robot_state_publisher du lanceur pour CE fichier, developpe ses parametres comme ros2 launch, rend la valeur de robot_description."""
    from launch import LaunchContext
    desc = _load()
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        "robot": "", "robot_config": "", "urdf": os.path.join(ROBOTS, urdf_file), "publish_joints": "false", "rviz": "false", "use_sim_time": "false"})
    os.environ["LINOROBOT2_BASE"] = "4wd"                                   # sans fichier canonique, le lanceur retombe sur la famille
    actions = desc._describe(ctx)
    node = [a for a in actions if "robot_state_publisher" in str(getattr(a, "node_executable", ""))][0]
    node._perform_substitutions(ctx)                                        # expansion, exactement comme au lancement
    params = node._Node__expanded_parameter_arguments
    path = [a for a in (params or []) if a][0][0]
    return list(yaml.safe_load(open(path)).values())[0]["ros__parameters"]["robot_description"]


@needs_xacro
@pytest.mark.parametrize("name", sorted(f for f in os.listdir(ROBOTS) if f.endswith(".urdf.xacro")))
def test_every_shipped_urdf_is_published_as_a_string(name):
    value = _expanded_robot_description(name)
    assert isinstance(value, str) and "<robot" in value and "</robot>" in value, name


@needs_xacro
def test_a_comment_with_a_colon_no_longer_breaks_the_description(tmp_path):
    """Le defaut d'origine, reproduit a volonte : un commentaire « cle: valeur » au milieu de l'URDF."""
    f = tmp_path / "piege.urdf.xacro"
    f.write_text('<?xml version="1.0"?>\n<robot name="piege" xmlns:xacro="http://ros.org/wiki/xacro">\n'
                 '  <!-- [mesure] lidar.pose_x : 4 cm en avant\n       autre: chose -->\n  <link name="base_link"/>\n</robot>\n', encoding="utf-8")
    value = _expanded_robot_description(str(f))
    assert "lidar.pose_x : 4 cm" in value


def test_the_launch_wraps_the_command_in_a_string_parameter_value():
    """Lecture directe du texte : si quelqu'un remet un Command nu, ce test echoue meme sans ROS."""
    text = open(LAUNCH, encoding="utf-8").read().replace("\r\n", "\n")
    assert "ParameterValue(Command(['xacro ', urdf] + xacro_args), value_type=str)" in text
    assert not re.search(r"'robot_description':\s*Command\(", text)
