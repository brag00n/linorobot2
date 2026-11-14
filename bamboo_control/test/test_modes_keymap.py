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
           "flip_image", "nav_unavailable", "acq", "trkv", "detector_prev",
           "tracker_prev", "predict_prev", "capture_next", "capture_prev",
           "calib_pid_raised", "calib_pid_ground",
           "calib_batt_discharge", "calib_batt_passive", "calib_batt_save",
           "calib_batt_stop", "nav_explore", "nav_auto_explore", "nav_save_map", "nav_idle",
           "nav_navigate", "nav_save_wp", "nav_goto_next", "nav_goto_home", "nav_cancel",
           "nav_new_map", "nav_gate_next", "nav_gate_prev", "nav_explore_gate",
           "nav_dest_next", "nav_dest_prev", "nav_goto_sel", "nav_dest_del", "nav_relocalize",
           "nav_map_next", "nav_map_prev", "nav_map_delete", "chain_all", "nav_toggle",
           "nav_patrol", "nav_dest_del_confirm"}


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
    # NAV demarre la navigation A LA DEMANDE (option B) ; les autres modes n'en ont pas.
    assert model["navigation"]["ondemand"] == "nav"
    assert model["conduite"]["ondemand"] == ""

    joy = _load("joy_bamboo.yaml", "bamboo_teleop")
    km = parseKeymap({k: v for k, v in joy.items() if k.startswith("keymap.")}, ACTIONS,
                     modes=names)
    assert km["calibration"]["bind"]["mesure"]["A_long"] == "calib_start"
    # Les anciennes fonctions sont toutes retrouvees dans un mode.
    bound = {a for m in km.values() for g in m["bind"].values() for a in g.values()}
    for old in ("tracking", "recenter", "recognition", "acquisition", "tracker", "predict",
                "detector", "training", "speed_max_up",
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
    with pytest.raises(ValueError):                       # groupe : pas de chemin ni d'espace
        parseModel(["a"], {"a": {"ondemand": "../nav"}})
    m = parseModel(["a"], {"a": {"enter_params": "/n:p=1;/n:q=true",
                                 "enter_modecmd": "tracking=off"}})
    assert m["a"]["enter_params"] == [("/n", "p", "1"), ("/n", "q", "true")]
    assert m["a"]["label"] == "A"


def test_display_key():
    assert displayKey("X_long") == "X long"
    assert displayKey("MINUS") == "-"


def test_battery_group_has_no_plein_key():
    """Le « plein » est fait par chaque decharge (repos initial) : plus de touche A long."""
    import re
    import os
    src = open(os.path.join(os.path.dirname(__file__), "..", "config", "joy_bamboo.yaml"),
               encoding="utf-8").read()
    assert "calib_batt_full" not in src
    bloc = re.search(r"batterie:\n((?:          .*\n)+)", src).group(1)
    assert "X_long" in bloc and "Y_long" in bloc and "R_long" in bloc and 'A_long: "calib_batt_save"' in bloc


# ---- combinaison ZL + ZR (2026-11-12) ----
def test_the_chord_fires_once_after_the_hold_time():
    from bamboo_control.keymap_model import Chord
    c = Chord(0.6)
    assert c.update(1, 1, 0.0) == (False, True)
    assert c.update(1, 1, 0.5) == (False, True)
    assert c.update(1, 1, 0.7) == (True, True)
    assert c.update(1, 1, 2.0) == (False, True)          # une seule fois par appui


def test_a_single_trigger_never_starts_the_chord_and_releasing_one_does_not_rearm():
    from bamboo_control.keymap_model import Chord
    c = Chord(0.6)
    assert c.update(0, 1, 0.0) == (False, False) and c.update(0, 1, 5.0) == (False, False)
    c.update(1, 1, 6.0); assert c.update(1, 1, 6.7)[0]
    assert c.update(1, 0, 6.8) == (False, True)           # ZL lache : les touches restent ignorees
    assert c.update(1, 1, 7.5) == (False, True)           # ...et ne redeclenchent pas
    assert c.update(0, 0, 8.0) == (False, False)
    c.update(1, 1, 9.0); assert c.update(1, 1, 9.7)[0]    # nouvel appui complet : redeclenche


def test_a_released_early_chord_does_not_fire():
    from bamboo_control.keymap_model import Chord
    c = Chord(0.6)
    c.update(1, 1, 0.0); c.update(1, 1, 0.3)
    assert c.update(0, 0, 0.4) == (False, False)
    assert c.update(1, 1, 1.0) == (False, True) and c.update(1, 1, 1.5) == (False, True)


def test_the_tracking_group_is_declared_for_the_host():
    path = os.path.join(HERE, "..", "..", "docker", "ondemand", "groups.conf")
    lines = [l.split("#")[0].split() for l in open(path, encoding="utf-8")]
    assert ["tracking", "bamboo_videotracking"] in lines
