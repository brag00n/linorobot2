"""Choix du fichier URDF par description.launch.py : l'argument de lancement, puis la cle `urdf:` du profil, puis la convention <famille>.urdf.xacro.

Le lanceur exige ROS (launch, ament) : on en EXTRAIT le bloc de decision et on l'execute sur de faux objets, comme les tests du teleop. Si le texte de ce bloc change
de forme, ce test echoue ici plutot que sur le robot.

    python3 -m pytest linorobot2_description/test_urdf_choice.py -q
"""
import os
import re
import textwrap

import pytest

HERE = os.path.dirname(__file__)
SRC = os.path.join(HERE, "launch", "description.launch.py")
YAMLS = {
    "WS": os.path.join(HERE, "..", "bamboo4WD_V4_WSEsp32_base", "config", "bamboo4WD_V4_WSEsp32.yaml"),
    "YB": os.path.join(HERE, "..", "bamboo4WD_V4_YBStm32_base", "config", "bamboo4WD_V4_YBStm32.yaml"),
}


def _decision():
    text = open(SRC, encoding="utf-8").read().replace("\r\n", "\n")
    m = re.search(r"(    urdf = LaunchConfiguration\('urdf'\)\.perform\(context\)\n.*?'urdf/robots', own or \(robot_base \+ '\.urdf\.xacro'\)\]\))", text, re.S)
    assert m, "bloc de choix de l'URDF introuvable dans description.launch.py"
    return textwrap.dedent(m.group(1))


class _Ctx:
    pass


def _choose(params, arg="", base="4wd"):
    class LC:
        def __init__(self, n):
            self.n = n

        def perform(self, _ctx):
            return arg

    ns = {"LaunchConfiguration": LC, "os": os, "params": params, "robot_base": base, "context": _Ctx(),
          "PathJoinSubstitution": lambda parts: "/".join(parts), "FindPackageShare": lambda p: "<share:%s>" % p}
    exec(_decision(), ns)                                                    # noqa: S102 - code du depot, extrait tel quel
    return ns["urdf"]


def test_without_a_key_the_family_convention_is_unchanged():
    assert _choose({}).endswith("/urdf/robots/4wd.urdf.xacro")


def test_the_profile_key_selects_its_own_file():
    assert _choose({"urdf": "bamboo_ws.urdf.xacro"}).endswith("/urdf/robots/bamboo_ws.urdf.xacro")


def test_the_launch_argument_still_has_the_last_word():
    assert _choose({"urdf": "bamboo_ws.urdf.xacro"}, arg="/tmp/essai.urdf") == "/tmp/essai.urdf"


@pytest.mark.parametrize("bad", ["../x.urdf", "/etc/passwd", "a/b.urdf.xacro", "..", "urdf/robots/x"])
def test_a_path_in_the_key_is_refused(bad):
    with pytest.raises(RuntimeError):
        _choose({"urdf": bad})


def test_bamboo_ws_urdf_exists_and_the_profile_keeps_the_family():
    """La cle `urdf:` du profil est ACTIVE ou COMMENTEE selon l'etat de l'activation : dans les deux cas le fichier existe et robot_base reste 4wd (le pilote le lit)."""
    import yaml
    own = yaml.safe_load(open(YAMLS["WS"], encoding="utf-8"))["/**"]["ros__parameters"]
    assert own["robot_base"] == "4wd"
    assert os.path.isfile(os.path.join(HERE, "urdf", "robots", own.get("urdf", "bamboo_ws.urdf.xacro")))
    if "urdf" in own:
        assert own["urdf"] == "bamboo_ws.urdf.xacro"


def test_the_other_robot_keeps_the_family_file():
    import yaml
    other = yaml.safe_load(open(YAMLS["YB"], encoding="utf-8"))["/**"]["ros__parameters"]
    assert "urdf" not in other
    assert os.path.isfile(os.path.join(HERE, "urdf", "robots", other["robot_base"] + ".urdf.xacro"))
