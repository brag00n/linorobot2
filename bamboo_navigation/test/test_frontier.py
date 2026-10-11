"""frontier.py sur des grilles synthetiques (5 cm la cellule), sans ROS."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

np = pytest.importorskip("numpy")
pytest.importorskip("yaml")
from bamboo_navigation import frontier  # noqa: E402

RES = 0.05


def room(w=60, h=40, open_side=None, gap=(15, 25)):
    """Piece de w x h cellules : murs (100) sur le pourtour, sol libre (0), inconnu (-1)
    au-dela. `open_side` = "right" ouvre le mur de droite sur [gap[0], gap[1][ en lignes."""
    g = np.full((h + 20, w + 20), -1, dtype=np.int16)
    g[10:10 + h, 10:10 + w] = 0
    g[10, 10:10 + w] = g[9 + h, 10:10 + w] = 100
    g[10:10 + h, 10] = g[10:10 + h, 9 + w] = 100
    if open_side == "right":
        g[10 + gap[0]:10 + gap[1], 9 + w] = 0
    return g


def test_closed_room_has_no_frontier():
    assert frontier.find_frontiers(room(), RES, (0.0, 0.0)) == []


def test_open_wall_gives_one_reachable_frontier():
    g = room(open_side="right", gap=(10, 30))           # ouverture de 1 m
    fs = frontier.find_frontiers(g, RES, (-1.0, -1.0))
    assert len(fs) == 1
    f = fs[0]
    assert f["size_m"] >= 0.9
    # point de visee : dans la piece, pres de l'ouverture, a distance des murs (0,30 m)
    j = int((f["y"] + 1.0) / RES)
    i = int((f["x"] + 1.0) / RES)
    assert g[j, i] == 0
    assert 10 + 10 <= j < 10 + 30
    assert 50 <= i <= 69


def test_narrow_opening_is_ignored():
    g = room(open_side="right", gap=(18, 22))           # 20 cm : sous le seuil de 30 cm
    assert frontier.find_frontiers(g, RES, (0.0, 0.0)) == []


def test_target_keeps_clearance_from_obstacles():
    g = room(open_side="right", gap=(10, 30))
    f = frontier.find_frontiers(g, RES, (0.0, 0.0), clearance_m=0.30)[0]
    occ = np.argwhere(g >= 65)
    j, i = int(f["y"] / RES), int(f["x"] / RES)
    dmin = np.sqrt(((occ - (j, i)) ** 2).sum(axis=1)).min() * RES
    assert dmin >= 0.30 - RES


def test_choose_prefers_big_and_near_and_avoids_tried_points():
    fs = [{"x": 1.0, "y": 0.0, "size_m": 1.0, "cells": 20},
          {"x": 5.0, "y": 0.0, "size_m": 1.0, "cells": 20},
          {"x": 1.2, "y": 0.0, "size_m": 0.4, "cells": 8}]
    assert frontier.choose(fs, (0.0, 0.0))["x"] == 1.0
    assert frontier.choose(fs, (0.0, 0.0), avoid=[(1.0, 0.0)])["x"] == 5.0
    assert frontier.choose(fs, (0.0, 0.0), avoid=[(1.0, 0.0), (5.0, 0.0)]) is None


def test_as_grid_matches_occupancygrid_layout():
    g = frontier.as_grid([0, 1, 2, 3, 4, 5], 3, 2)
    assert g.shape == (2, 3) and g[1, 0] == 3           # ligne 1 = y superieur
