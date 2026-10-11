"""Frontieres d'exploration sur une grille d'occupation (numpy, sans ROS).

Une FRONTIERE est la limite entre le connu et l'inconnu : une cellule LIBRE dont un voisin
(4-connexite) est INCONNU. Explorer, c'est aller de frontiere en frontiere jusqu'a ce qu'il
n'en reste plus ; c'est la strategie retenue pour l'exploration automatique (2026-10-11).

Conventions de map_io / map_server : valeur < 0 = inconnu, <= FREE_THRESH*100 = libre,
>= OCC_THRESH*100 = occupe. La grille est rangee ligne par ligne a partir de y MIN (la
ligne 0 est en bas), comme le `data` d'un nav_msgs/OccupancyGrid ; l'origine est le coin de
la cellule (0, 0), sans rotation (slam_toolbox et map_server n'en mettent pas).
"""
import math
from collections import deque

import numpy as np

from bamboo_navigation.map_io import FREE_THRESH, OCC_THRESH


def as_grid(data, width, height):
    """`data` (liste ou tableau a plat) -> tableau (hauteur, largeur) d'entiers."""
    return np.asarray(data, dtype=np.int16).reshape(height, width)


def _dilate(mask, r):
    """Dilatation binaire par un disque de rayon `r` cellules (decalages numpy, sans scipy)."""
    out = mask.copy()
    h, w = mask.shape
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if (dx or dy) and dx * dx + dy * dy <= r * r:
                ys, yd = (slice(dy, h), slice(0, h - dy)) if dy >= 0 else \
                    (slice(0, h + dy), slice(-dy, h))
                xs, xd = (slice(dx, w), slice(0, w - dx)) if dx >= 0 else \
                    (slice(0, w + dx), slice(-dx, w))
                out[yd, xd] |= mask[ys, xs]
    return out


def find_frontiers(grid, res, origin, min_size_m=0.30, clearance_m=0.30, search_m=0.60):
    """Frontieres de la grille, chacune avec son POINT DE VISEE.

    Retourne une liste de dicts {x, y, size_m, cells} : (x, y) est une cellule LIBRE a au
    moins `clearance_m` de tout obstacle (rayon d'inflation de Nav2 : sinon le but tombe dans
    la zone interdite et Nav2 le refuse), la plus proche du centre de l'amas, cherchee a
    moins de `search_m` de lui. Un amas plus court que `min_size_m`, ou sans point de visee
    accessible, est ecarte : c'est souvent du bruit du lidar ou un recoin sous un meuble.
    """
    g = np.asarray(grid)
    h, w = g.shape
    free = (g >= 0) & (g <= FREE_THRESH * 100)
    unknown = g < 0
    occ = g >= OCC_THRESH * 100
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    front = free & near_unknown

    blocked = _dilate(occ, int(math.ceil(clearance_m / res)))
    target_ok = free & ~blocked
    min_cells = max(1, int(round(min_size_m / res)))
    rs = int(math.ceil(search_m / res))

    seen = np.zeros_like(front)
    out = []
    for j0, i0 in zip(*np.nonzero(front)):
        if seen[j0, i0]:
            continue
        cells, queue = [], deque([(j0, i0)])
        seen[j0, i0] = True
        while queue:                                   # amas en 8-connexite
            j, i = queue.popleft()
            cells.append((j, i))
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    jj, ii = j + dj, i + di
                    if 0 <= jj < h and 0 <= ii < w and front[jj, ii] and not seen[jj, ii]:
                        seen[jj, ii] = True
                        queue.append((jj, ii))
        if len(cells) < min_cells:
            continue
        cj = sum(c[0] for c in cells) / len(cells)
        ci = sum(c[1] for c in cells) / len(cells)
        j_lo, j_hi = max(0, int(cj) - rs), min(h, int(cj) + rs + 1)
        i_lo, i_hi = max(0, int(ci) - rs), min(w, int(ci) + rs + 1)
        cand = np.argwhere(target_ok[j_lo:j_hi, i_lo:i_hi])
        if cand.size == 0:
            continue
        cand = cand + (j_lo, i_lo)
        d2 = (cand[:, 0] - cj) ** 2 + (cand[:, 1] - ci) ** 2
        j, i = cand[int(np.argmin(d2))]
        if d2.min() > rs * rs:
            continue
        out.append({"x": origin[0] + (i + 0.5) * res, "y": origin[1] + (j + 0.5) * res,
                    "size_m": len(cells) * res, "cells": len(cells)})
    return out


def choose(frontiers, robot_xy, avoid=(), avoid_r=0.5):
    """La frontiere a viser : grande et proche (score = taille / (distance + 0,5 m)).

    `avoid` = points deja tentes (echecs ou buts atteints qui sont restes des frontieres) :
    une frontiere a moins de `avoid_r` de l'un d'eux est ignoree, sinon le robot y
    reviendrait en boucle. None s'il ne reste rien : l'exploration est terminee.
    """
    best, best_score = None, -1.0
    rx, ry = robot_xy
    for f in frontiers:
        if any(math.hypot(f["x"] - ax, f["y"] - ay) < avoid_r for ax, ay in avoid):
            continue
        score = f["size_m"] / (math.hypot(f["x"] - rx, f["y"] - ry) + 0.5)
        if score > best_score:
            best, best_score = f, score
    return best
