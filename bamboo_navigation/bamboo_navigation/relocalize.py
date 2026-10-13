"""Relocalisation GLOBALE sur une carte, et controle « la carte correspond a ce que voit le
lidar » (numpy, sans ROS ; teste hors robot).

POURQUOI : slam_toolbox (choisi pour toute la navigation, 2026-10-12) ne sait se localiser
qu'a partir d'une pose APPROCHEE (map_start_pose, /initialpose) ; il n'a pas de localisation
globale. Le robot pose n'importe ou doit donc d'abord etre retrouve ici, puis cette pose est
donnee a slam_toolbox, qui affine et suit.

METHODE (champ de vraisemblance, comme le « likelihood field » d'AMCL) :
  - pour chaque cellule de la carte, la DISTANCE a l'obstacle le plus proche (plafonnee) ;
  - le SCORE d'une pose = moyenne, sur les points du scan places a cette pose, de
    exp(-d^2 / 2 sigma^2) : 1 = chaque point tombe sur un mur de la carte ; un point qui
    tombe dans l'INCONNU vaut `unknown` (ni preuve, ni dementi) ;
  - recherche GLOBALE : toutes les cellules libres (pas `step`), `n_head` caps, avec un sigma
    LARGE (la grille de recherche est grossiere), puis AFFINAGE des meilleures (descente de
    pas, sigma fin) ;
  - COHERENCE des rayons : la note finale d'une hypothese affinee est multipliee par la part
    des rayons qui ne TRAVERSENT aucun mur de la carte avant leur impact (judge) ;
  - DECISION : la meilleure est acceptee si son score atteint `accept` ET si elle devance de
    `margin` la meilleure hypothese DISTINCTE (ailleurs, ou meme endroit mais autre cap :
    une piece rectangulaire vide se lit pareil retournee de 180 degres) ;
  - AMBIGUITE : le superviseur fait tourner le robot et ajoute un scan (Relocalizer.update) ;
    chaque hypothese suit le deplacement mesure par l'odometrie et cumule ses scores.
"""
import math

import numpy as np

OCC, FREE = 65, 25          # seuils OccupancyGrid (0..100) : occupe >= 65, libre <= 25


def as_array(data, width, height):
    """OccupancyGrid.data -> tableau (hauteur, largeur), ligne 0 = y MIN (en bas)."""
    return np.asarray(data, dtype=np.int16).reshape(height, width)


def distance_field(occ, res, max_d):
    """Distance (m) de chaque cellule a la cellule occupee la plus proche, plafonnee a
    `max_d`. Chanfrein 8-voisins itere (erreur < 8 % de la distance euclidienne), sans
    scipy (absent de l'image du robot)."""
    d = np.where(occ, 0.0, max_d).astype(np.float32)
    h, w = d.shape
    diag = math.sqrt(2.0) * res
    for _ in range(int(math.ceil(max_d / res)) + 1):
        n = d.copy()
        for dy, dx, step in ((0, 1, res), (0, -1, res), (1, 0, res), (-1, 0, res),
                             (1, 1, diag), (1, -1, diag), (-1, 1, diag), (-1, -1, diag)):
            ys, yd = slice(max(dy, 0), h + min(dy, 0)), slice(max(-dy, 0), h + min(-dy, 0))
            xs, xd = slice(max(dx, 0), w + min(dx, 0)), slice(max(-dx, 0), w + min(-dx, 0))
            np.minimum(n[yd, xd], d[ys, xs] + step, out=n[yd, xd])
        if np.array_equal(n, d):
            break
        d = n
    return np.minimum(d, max_d)


def scan_points(ranges, angle_min, angle_inc, range_min=0.05, range_max=8.0,
                laser=(0.0, 0.0, 0.0), max_n=90):
    """Points d'un LaserScan dans le repere du ROBOT (base_footprint), (N, 2). `laser` = pose
    (x, y, cap) du lidar sur le robot. Au plus `max_n` points, repartis sur le tour."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    r, a = r[ok], a[ok]
    if len(r) > max_n:
        idx = np.linspace(0, len(r) - 1, max_n).astype(int)
        r, a = r[idx], a[idx]
    lx, ly = r * np.cos(a), r * np.sin(a)
    c, s = math.cos(laser[2]), math.sin(laser[2])
    return np.stack([laser[0] + c * lx - s * ly, laser[1] + s * lx + c * ly], axis=1)


def angdiff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def compose(pose, delta):
    """pose (x, y, cap) suivie d'un deplacement `delta` exprime dans le repere du robot."""
    x, y, t = pose
    dx, dy, dt = delta
    c, s = math.cos(t), math.sin(t)
    return (x + c * dx - s * dy, y + s * dx + c * dy, angdiff(t + dt, 0.0))


def relative(p0, p1):
    """Deplacement de p0 a p1 (deux poses odometriques), dans le repere du robot en p0."""
    c, s = math.cos(p0[2]), math.sin(p0[2])
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return (c * dx + s * dy, -s * dx + c * dy, angdiff(p1[2], p0[2]))


class LikelihoodMap:
    """Une carte prete a SCORER des poses (champ de distance calcule une fois)."""

    def __init__(self, grid, res, origin, max_d=0.5, unknown=0.25, clearance=0.10):
        self.grid = np.asarray(grid)
        self.res = float(res)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.h, self.w = self.grid.shape
        occ = self.grid >= OCC
        self.known = self.grid >= 0
        self.free = self.known & (self.grid <= FREE)
        self.dist = distance_field(occ, self.res, max_d)
        self.max_d = max_d
        self.unknown = unknown
        self.clearance = clearance
        self._lik = {}

    @classmethod
    def from_occupancy(cls, data, width, height, res, origin, **kw):
        return cls(as_array(data, width, height), res, origin, **kw)

    def lik(self, sigma):
        """Table de vraisemblance pour `sigma` : gaussienne de la distance ; dans l'inconnu,
        au moins `unknown` (un mur pas encore cartographie n'est pas un dementi)."""
        t = self._lik.get(sigma)
        if t is None:
            t = np.exp(-(self.dist.astype(np.float64) ** 2) / (2.0 * sigma * sigma))
            t = np.where(self.known, t, np.maximum(t, self.unknown))
            self._lik[sigma] = t
        return t

    def score(self, pts, poses, sigma=0.05):
        """Score de chaque pose (M, 3) pour les points (N, 2) du robot : (M,)."""
        poses = np.atleast_2d(np.asarray(poses, dtype=np.float64))
        lik = self.lik(sigma)
        c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        px, py = pts[None, :, 0], pts[None, :, 1]
        wx = poses[:, 0:1] + c * px - s * py
        wy = poses[:, 1:2] + s * px + c * py
        ix = np.floor((wx - self.ox) / self.res).astype(np.int64)
        iy = np.floor((wy - self.oy) / self.res).astype(np.int64)
        inside = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        val = np.full(ix.shape, self.unknown)
        val[inside] = lik[iy[inside], ix[inside]]
        return val.mean(axis=1)

    def crossing(self, pts, pose, margin=0.30, depth=3):
        """Part des rayons qui TRAVERSENT un obstacle de la carte avant leur point d'impact
        (plus de `margin` avant lui) : un mur entre le lidar et ce qu'il a touche contredit la
        pose. Le seul champ de vraisemblance ne le voit pas -- essai G0 sur scan reel
        (2026-10-13) : des poses FAUSSES y scoraient 0,69 contre 0,72 pour la candidate juste,
        dans des pieces aux murs droits. TRAVERSER = `depth` echantillons occupes d'affilee
        (pas d'une demi-cellule, ~0,1 m) : un rayon qui RASE un angle de mur ne compte pas."""
        x, y, t = pose
        c, s = math.cos(t), math.sin(t)
        wx = x + c * pts[:, 0] - s * pts[:, 1]
        wy = y + s * pts[:, 0] + c * pts[:, 1]
        dx, dy = wx - x, wy - y
        r = np.hypot(dx, dy)
        n = int(np.max(r) / (self.res / 2.0)) + 1 if len(r) else 0
        if n <= depth:
            return 0.0
        f = np.linspace(0.0, 1.0, n)[None, :]
        sx = x + dx[:, None] * f
        sy = y + dy[:, None] * f
        along = r[:, None] * f
        use = (along > 0.10) & (along < r[:, None] - margin)
        ix = np.floor((sx - self.ox) / self.res).astype(np.int64)
        iy = np.floor((sy - self.oy) / self.res).astype(np.int64)
        inside = use & (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        occ = np.zeros(ix.shape, dtype=bool)
        occ[inside] = self.grid[iy[inside], ix[inside]] >= OCC
        run = occ[:, :n - depth + 1].copy()
        for k in range(1, depth):
            run &= occ[:, k:n - depth + 1 + k]
        return float(run.any(axis=1).mean())

    def judge(self, pts, pose, sigma=0.05):
        """Note FINALE d'une pose : vraisemblance x (1 - part des rayons qui traversent)."""
        return float(self.score(pts, [pose], sigma)[0]) * (1.0 - self.crossing(pts, pose))

    def candidates(self, step=0.15, region=None):
        """Positions (x, y) ou le robot TIENT : cellules libres a plus de `clearance` d'un
        obstacle, une tous les `step` m ; `region` = (x, y, rayon) pour restreindre."""
        k = max(1, int(round(step / self.res)))
        ok = self.free & (self.dist >= self.clearance)
        iy, ix = np.nonzero(ok[::k, ::k])
        x = self.ox + (ix * k + 0.5) * self.res
        y = self.oy + (iy * k + 0.5) * self.res
        if region is not None:
            keep = np.hypot(x - region[0], y - region[1]) <= region[2]
            x, y = x[keep], y[keep]
        return np.stack([x, y], axis=1)

    def refine(self, pts, pose, sigmas=(0.12, 0.05), xy0=0.08, th0=math.radians(4.0)):
        """Affinage local (descente de pas sur x, y, cap) ; rend (score, (x, y, cap))."""
        p = np.asarray(pose, dtype=np.float64)
        best = 0.0
        g = np.array([-1.0, 0.0, 1.0])
        steps = np.stack(np.meshgrid(g, g, g, indexing="ij"), axis=-1).reshape(-1, 3)
        for sigma in sigmas:
            sxy, sth = xy0, th0
            best = float(self.score(pts, p, sigma)[0])
            for _ in range(60):
                cand = p + steps * np.array([sxy, sxy, sth])
                sc = self.score(pts, cand, sigma)
                i = int(np.argmax(sc))
                if sc[i] > best + 1e-9:
                    best, p = float(sc[i]), cand[i]
                else:
                    sxy, sth = sxy / 2.0, sth / 2.0
                    if sxy < 0.005:
                        break
        return best, (float(p[0]), float(p[1]), angdiff(float(p[2]), 0.0))

    def global_search(self, pts, step=0.15, n_head=32, keep=12, region=None,
                      sigma_coarse=0.15, chunk_elems=2000000):
        """Hypotheses [(score, (x, y, cap))] triees, AFFINEES, sans doublons."""
        cand = self.candidates(step, region)
        if len(cand) == 0 or len(pts) == 0:
            return []
        heads = np.arange(n_head) * (2.0 * math.pi / n_head)
        per = max(1, chunk_elems // (n_head * len(pts)))
        coarse = []
        for i in range(0, len(cand), per):
            c = cand[i:i + per]
            poses = np.concatenate([np.repeat(c, n_head, axis=0),
                                    np.tile(heads, len(c))[:, None]], axis=1)
            coarse.append((self.score(pts, poses, sigma_coarse), poses))
        sc = np.concatenate([s for s, _ in coarse])
        poses = np.concatenate([p for _, p in coarse])
        order = np.argsort(-sc)
        picked = []
        for i in order:                    # suppression des non-maxima : 0,3 m / 25 degres
            p = poses[i]
            if all(not _same(p, q, 0.3, math.radians(25.0)) for q in picked):
                picked.append(p)
                if len(picked) >= 3 * keep:
                    break
        out = []
        for p in picked:
            _s, q = self.refine(pts, p)
            out.append((self.judge(pts, q), q))
        if region is not None:
            # L'affinage peut faire glisser une hypothese faible hors de la zone demandee :
            # une recherche LOCALE ne rend que des poses de sa zone.
            out = [h for h in out
                   if math.hypot(h[1][0] - region[0], h[1][1] - region[1]) <= region[2] + step]
        return dedupe(sorted(out, key=lambda h: -h[0]))[:keep]


def _same(p, q, dxy, dth):
    return math.hypot(p[0] - q[0], p[1] - q[1]) < dxy and abs(angdiff(p[2], q[2])) < dth


def dedupe(hyps, dxy=0.15, dth=math.radians(8.0)):
    out = []
    for h in hyps:
        if all(not _same(h[1], o[1], dxy, dth) for o in out):
            out.append(h)
    return out


def decide(hyps, accept=0.55, margin=0.08, distinct_m=0.5, distinct_rad=0.5):
    """(acceptee ?, meilleure, rivale) ; rivale = meilleure hypothese DISTINCTE de la
    premiere (a plus de `distinct_m`, ou meme endroit mais cap ecarte de `distinct_rad`)."""
    if not hyps:
        return False, None, None
    best = hyps[0]
    rival = next((h for h in hyps[1:]
                  if math.hypot(h[1][0] - best[1][0], h[1][1] - best[1][1]) > distinct_m or
                  abs(angdiff(h[1][2], best[1][2])) > distinct_rad), None)
    ok = best[0] >= accept and (rival is None or best[0] - rival[0] >= margin)
    return ok, best, rival


def pick_map(results, margin=0.10):
    """Parmi plusieurs CARTES, laquelle reconnait l'endroit ? `results` = [(nom, acceptee,
    meilleure, rivale)] dans l'ordre de preference (la plus RECENTE d'abord). Rend
    ((nom, meilleure), "") ou (None, raison).

    Seules comptent les cartes ou la pose a ete ACCEPTEE (seuil et marge sur sa propre
    carte). Plusieurs cartes de la MEME piece se valent souvent (versions successives) : parmi
    celles a moins de `margin` de la meilleure note, la premiere de la liste (la plus
    recente, en general la meilleure) l'emporte."""
    acc = [(k, n, b) for k, (n, ok, b, _r) in enumerate(results) if ok and b is not None]
    if not acc:
        tried = len(results)
        return None, "aucune des %d autre(s) carte(s) ne reconnait l'endroit" % tried
    top = max(b[0] for _k, _n, b in acc)
    close = [(k, n, b) for k, n, b in acc if top - b[0] < margin]
    _k, name, best = min(close, key=lambda c: c[0])
    return (name, best), ""


class Relocalizer:
    """Plusieurs scans pour lever une AMBIGUITE : chaque hypothese suit le deplacement de
    l'odometrie (rotation sur place surtout), est re-affinee sur le nouveau scan et garde
    la MOYENNE de ses scores. La vraie pose reste haute ; une fausse, vue d'un autre angle,
    retombe."""

    def __init__(self, lmap, accept=0.55, margin=0.08, keep=12):
        self.m = lmap
        self.accept, self.margin, self.keep = accept, margin, keep
        self.hyps = []                    # [(moyenne, pose, [scores])]
        self.scans = 0

    def start(self, pts, region=None, step=0.15, n_head=32):
        hs = self.m.global_search(pts, step=step, n_head=n_head, keep=self.keep,
                                  region=region)
        self.hyps = [(s, p, [s]) for s, p in hs]
        self.scans = 1
        return self.decision()

    def update(self, pts, delta):
        out = []
        for _mean, pose, scores in self.hyps:
            _s, p = self.m.refine(pts, compose(pose, delta))
            sc = scores + [self.m.judge(pts, p)]
            out.append((sum(sc) / len(sc), p, sc))
        out.sort(key=lambda h: -h[0])
        kept = []
        for h in out:                      # deux hypotheses qui convergent n'en font qu'une
            if all(not _same(h[1], k[1], 0.15, math.radians(8.0)) for k in kept):
                kept.append(h)
        self.hyps = kept
        self.scans += 1
        return self.decision()

    def decision(self):
        return decide([(m, p) for m, p, _ in self.hyps], self.accept, self.margin)


def raycast(lmap, pose, angles, range_max=8.0):
    """Portees qu'un lidar parfait mesurerait a `pose` sur la carte (essais hors robot) :
    premier point occupe le long de chaque rayon, inf si rien avant `range_max`."""
    t = np.arange(0.0, range_max, lmap.res / 2.0)
    a = pose[2] + np.asarray(angles, dtype=np.float64)
    wx = pose[0] + np.cos(a)[:, None] * t[None, :]
    wy = pose[1] + np.sin(a)[:, None] * t[None, :]
    ix = np.floor((wx - lmap.ox) / lmap.res).astype(np.int64)
    iy = np.floor((wy - lmap.oy) / lmap.res).astype(np.int64)
    inside = (ix >= 0) & (ix < lmap.w) & (iy >= 0) & (iy < lmap.h)
    hit = np.zeros(ix.shape, dtype=bool)
    hit[inside] = lmap.grid[iy[inside], ix[inside]] >= OCC
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    return np.where(first >= 0, t[np.maximum(first, 0)], np.inf)
