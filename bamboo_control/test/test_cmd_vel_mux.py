"""Coeur du multiplexeur de consignes, sans ROS."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from bamboo_control.cmd_vel_mux_node import Mux  # noqa: E402


def test_gamepad_overrides_navigation_then_hands_back():
    m = Mux({"teleop": (100, 0.5), "calib": (90, 0.5), "nav": (10, 0.5)})
    assert m.offer("nav", 0.0)                 # seule : la navigation pilote
    assert m.offer("teleop", 0.1)              # stick pousse : la manette prend la main
    assert not m.offer("nav", 0.2)             # la navigation est muette en sortie
    assert not m.offer("nav", 0.55)            # manette encore fraiche (0,45 s)
    assert m.offer("nav", 0.65)                # manette muette depuis 0,55 s : nav reprend
    assert m.active(5.0) is None               # tout le monde se tait


def test_priority_order():
    m = Mux({"teleop": (100, 0.5), "calib": (90, 0.5), "nav": (10, 0.5)})
    m.offer("nav", 0.0)
    assert m.offer("calib", 0.1)
    assert not m.offer("nav", 0.2)
    assert m.offer("teleop", 0.3)
    assert not m.offer("calib", 0.35)


def test_gated_source_passes_only_while_enabled():
    m = Mux({"teleop": (100, 0.5), "nav": (10, 0.5)}, gated=["nav"])
    assert not m.offer("nav", 0.0)             # jamais autorisee : fermee
    assert not m.set_enabled("nav", True, 0.1)
    assert m.offer("nav", 0.2)                 # autorisee : elle pilote
    assert m.set_enabled("nav", False, 0.3)    # coupee alors qu'elle avait la main -> STOP
    assert not m.offer("nav", 0.35)            # residus de Nav2 apres l'arret : ignores
    assert m.active(0.4) is None
    assert m.offer("teleop", 0.5)              # la manette, elle, n'a pas de verrou
