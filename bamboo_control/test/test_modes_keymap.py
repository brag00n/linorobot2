"""Modele des modes (modes.yaml) et carte des touches (joy_bamboo.yaml), sans ROS.

Les deux fichiers LIVRES sont relus et valides tels quels : une faute de frappe dans l'un
doit casser ce test, pas le premier appui sur la croix.
"""
import os
import sys

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))

from bamboo_control.keymap_model import displayKey, parseKeymap  # noqa: E402
from bamboo_control.modes_model import FIELDS, parseModel  # noqa: E402

# Doit rester aligne sur TeleopNode._buildActions.
ACTIONS = {"tracking", "recenter", "recognition", "acquisition", "tracker", "predict",
           "detector", "training", "target_grow", "target_shrink", "speed_max_up",
           "speed_max_down", "speed_min_up", "speed_min_down", "calib_start", "calib_imu",
           "calib_arcs", "calib_variant", "calib_apply", "calib_revert", "calib_pi",
           "flip_image", "nav_unavailable"}


def _flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        name = prefix + str(k)
        if isinstance(v, dict):
            out.update(_flatten(v, name + "."))
        else:
            out[name] = v
    return out


def _load(rel, node):
    yaml = pytest.importorskip("yaml")
    with open(os.path.join(HERE, "..", "config", rel), encoding="utf-8") as fh:
        return _flatten(yaml.safe_load(fh)[node]["ros__parameters"])


def test_shipped_files_are_valid_and_consistent():
    modes = _load("modes.yaml", "bamboo_mode_manager")
    names = modes["modes"]
    raw = {n: {f: modes.get("%s.%s" % (n, f), "") for f in FIELDS} for n in names}
    model = parseModel(names, raw)
    assert modes["default_mode"] in model
    assert model["calibration"]["exit_cancel"] == ["/bamboo_calibration/calibrate"]

    joy = _load("joy_bamboo.yaml", "bamboo_teleop")
    km = parseKeymap({k: v for k, v in joy.items() if k.startswith("keymap.")}, ACTIONS,
                     modes=names)
    assert km["calibration"]["bind"]["mesure"]["A_long"] == "calib_start"
    # Les anciennes fonctions sont toutes retrouvees dans un mode.
    bound = {a for m in km.values() for g in m["bind"].values() for a in g.values()}
    for old in ("tracking", "recenter", "recognition", "acquisition", "tracker", "predict",
                "detector", "training", "target_grow", "target_shrink", "speed_max_up",
                "speed_max_down", "speed_min_up", "speed_min_down"):
        assert old in bound, old


def test_keymap_rejects_global_unknown_and_orphans():
    base = {"keymap.m.groups": "g"}
    with pytest.raises(ValueError, match="GLOBALE"):
        parseKeymap(dict(base, **{"keymap.m.g.B": "tracking"}), ACTIONS)
    with pytest.raises(ValueError, match="touche inconnue"):
        parseKeymap(dict(base, **{"keymap.m.g.Q": "tracking"}), ACTIONS)
    with pytest.raises(ValueError, match="action inconnue"):
        parseKeymap(dict(base, **{"keymap.m.g.A": "voler"}), ACTIONS)
    with pytest.raises(ValueError, match="groupe"):
        parseKeymap(dict(base, **{"keymap.m.h.A": "tracking"}), ACTIONS)
    with pytest.raises(ValueError, match="modes inconnus"):
        parseKeymap(dict(base, **{"keymap.m.g.A": "tracking"}), ACTIONS, modes=["x"])


def test_model_rejects_bad_entries():
    with pytest.raises(ValueError):
        parseModel(["a"], {"a": {"requires": "armed"}})
    with pytest.raises(ValueError):
        parseModel(["a"], {"a": {"enter_modecmd": "tracking="}})
    with pytest.raises(ValueError):
        parseModel(["a"], {"a": {"enter_params": "noeud:p=1"}})
    with pytest.raises(ValueError):
        parseModel(["a", "a"], {})
    m = parseModel(["a"], {"a": {"enter_params": "/n:p=1;/n:q=true",
                                 "enter_modecmd": "tracking=off"}})
    assert m["a"]["enter_params"] == [("/n", "p", "1"), ("/n", "q", "true")]
    assert m["a"]["label"] == "A"


def test_display_key():
    assert displayKey("X_long") == "X long"
    assert displayKey("MINUS") == "-"
