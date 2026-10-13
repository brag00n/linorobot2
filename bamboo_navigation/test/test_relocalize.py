"""Relocalisation globale (numpy, sans ROS) sur des cartes de synthese."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

np = pytest.importorskip("numpy")
from bamboo_navigation import relocalize as rl  # noqa: E402

RES = 0.05


def _room(w_m, h_m, extra=()):
    """Piece fermee w x h (m), murs d'une cellule, origine (0, 0) ; `extra` = rectangles
    occupes (x0, y0, x1, y1) en m. Libre partout ailleurs."""
    w, h = int(w_m / RES), int(h_m / RES)
    g = np.zeros((h, w), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    for x0, y0, x1, y1 in extra:
        g[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = 100
    return g


def _l_room():
    # 6 x 4 m ; un bloc occupe le coin haut-droit (forme en L), une caisse, une cloison.
    return _room(6.0, 4.0, [(3.5, 2.5, 6.0, 4.0), (1.0, 1.0, 1.4, 1.3), (4.5, 0.0, 4.6, 1.2)])


def _scan(lmap, pose, n=360, noise=0.01, seed=0):
    ang = np.arange(n) * 2.0 * math.pi / n - math.pi
    r = rl.raycast(lmap, pose, ang)
    r = r + np.random.default_rng(seed).normal(0.0, noise, r.shape)
    return rl.scan_points(r, -math.pi, 2.0 * math.pi / n)


def test_distance_field_is_close_to_euclidean():
    occ = np.zeros((41, 41), dtype=bool)
    occ[20, 20] = True
    d = rl.distance_field(occ, RES, 0.5)
    assert d[20, 20] == 0.0
    assert abs(d[20, 28] - 0.40) < 1e-6                     # en ligne droite : exact
    assert abs(d[26, 26] - math.hypot(0.3, 0.3)) < 0.02     # en diagonale : chanfrein
    assert d[0, 0] == pytest.approx(0.5)                    # plafonne


def test_score_peaks_at_the_true_pose():
    m = rl.LikelihoodMap(_l_room(), RES, (0.0, 0.0))
    true = (2.0, 1.8, 0.7)
    pts = _scan(m, true)
    s_true = m.score(pts, [true])[0]
    s_off = m.score(pts, [(2.5, 1.8, 0.7), (2.0, 1.8, 1.2)])
    assert s_true > 0.8 and (s_off < s_true - 0.3).all()


def test_global_localization_finds_the_pose():
    m = rl.LikelihoodMap(_l_room(), RES, (0.0, 0.0))
    for true in ((2.0, 1.8, 0.7), (5.0, 1.5, -2.5), (0.8, 3.2, 3.0)):
        r = rl.Relocalizer(m)
        ok, best, rival = r.start(_scan(m, true))
        assert ok, (true, best, rival)
        x, y, t = best[1]
        assert math.hypot(x - true[0], y - true[1]) < 0.10, (true, best)
        assert abs(rl.angdiff(t, true[2])) < math.radians(5.0), (true, best)


def test_symmetric_room_is_ambiguous():
    # Rectangle vide : la pose et sa symetrique (centre, cap + 180 deg) se lisent pareil.
    m = rl.LikelihoodMap(_room(5.0, 3.0), RES, (0.0, 0.0))
    ok, best, rival = rl.Relocalizer(m).start(_scan(m, (1.5, 1.0, 0.3)))
    assert not ok and rival is not None
    assert abs(best[0] - rival[0]) < 0.08


def test_second_scan_after_turning_keeps_the_true_pose():
    m = rl.LikelihoodMap(_l_room(), RES, (0.0, 0.0))
    true = (2.0, 1.8, 0.7)
    r = rl.Relocalizer(m)
    r.start(_scan(m, true))
    turned = (true[0], true[1], true[2] + math.pi / 2)
    ok, best, _ = r.update(_scan(m, turned, seed=1), (0.0, 0.0, math.pi / 2))
    assert ok and len(best) == 2
    assert math.hypot(best[1][0] - 2.0, best[1][1] - 1.8) < 0.10
    assert abs(rl.angdiff(best[1][2], turned[2])) < math.radians(5.0)


def test_local_search_stays_in_its_region():
    m = rl.LikelihoodMap(_l_room(), RES, (0.0, 0.0))
    hs = m.global_search(_scan(m, (2.0, 1.8, 0.7)), region=(2.2, 1.6, 0.6))
    assert hs and all(math.hypot(p[0] - 2.2, p[1] - 1.6) < 0.6 + 0.2 for _s, p in hs)


def test_rays_through_a_wall_condemn_a_pose():
    # Deux pieces identiques cote a cote, separees par un mur ; la vraie pose est dans celle
    # de gauche, la porte de communication est fermee. Placee a droite, la meme lecture
    # colle aussi aux murs, mais ses rayons vers la gauche TRAVERSENT le mur mitoyen.
    g = _room(8.0, 3.0, [(3.95, 0.0, 4.05, 3.0)])
    m = rl.LikelihoodMap(g, RES, (0.0, 0.0))
    pts = _scan(m, (1.0, 1.5, 0.0))
    assert m.crossing(pts, (1.0, 1.5, 0.0)) < 0.02
    assert m.crossing(pts, (5.0, 1.5, 0.0)) < 0.02          # piece jumelle : rien a redire
    wrong = (3.0, 1.5, 0.0)                                 # decalee : rayons vers la droite
    assert m.crossing(pts, wrong) > 0.1
    assert m.judge(pts, wrong) < m.judge(pts, (1.0, 1.5, 0.0)) - 0.2


def test_pick_map_among_several():
    p = (1.0, 2.0, 0.0)
    # Aucune carte acceptee : refus.
    assert rl.pick_map([("a", False, (0.5, p), None), ("b", False, None, None)])[0] is None
    # Une seule acceptee : elle, meme si une autre (refusee) note presque autant.
    got, _why = rl.pick_map([("a", False, (0.60, p), None), ("b", True, (0.62, p), None)])
    assert got[0] == "b"
    # Deux versions de la meme piece, notes voisines : la plus RECENTE (premiere) l'emporte.
    got, _why = rl.pick_map([("recente", True, (0.70, p), None),
                             ("ancienne", True, (0.75, p), None)])
    assert got[0] == "recente"
    # Ecart net : la meilleure l'emporte, meme plus ancienne.
    got, _why = rl.pick_map([("recente", True, (0.58, p), None),
                             ("ancienne", True, (0.90, p), None)])
    assert got[0] == "ancienne"


def test_compose_and_relative_are_inverse():
    p0, p1 = (1.0, 2.0, 0.5), (1.4, 2.3, 1.1)
    back = rl.compose(p0, rl.relative(p0, p1))
    assert all(abs(a - b) < 1e-9 for a, b in zip(back, p1))


def test_unknown_area_is_neutral():
    g = _l_room()
    g[:, 80:] = -1                       # la moitie droite n'a jamais ete vue
    m = rl.LikelihoodMap(g, RES, (0.0, 0.0), unknown=0.25)
    pts = _scan(rl.LikelihoodMap(_l_room(), RES, (0.0, 0.0)), (2.0, 1.8, 0.7))
    assert m.score(pts, [(2.0, 1.8, 0.7)])[0] > 0.45
