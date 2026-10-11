"""Cartes et points de navigation sur disque (Python pur, format map_server).

Une carte = <nom>.pgm (P5, 8 bits) + <nom>.yaml (map_server), dans le dossier `maps` du
paquet du robot (<robot>_base/maps). Ses points de navigation = <nom>_waypoints.yaml :
  waypoints:
    - {name: depart, x: 0.0, y: 0.0, yaw: 0.0}
L'ecriture de la carte ne passe PAS par nav2_map_saver : on a deja la grille (abonnement
au sujet `map`), et l'ecrire soi-meme evite un noeud de plus et un service a attendre.
"""
import datetime
import math
import os

import yaml

FREE_THRESH, OCC_THRESH = 0.25, 0.65


def stamp_name(prefix="carte"):
    return "%s_%s" % (prefix, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))


def write_map(base, width, height, resolution, origin_xyz_yaw, data):
    """Ecrit base.pgm + base.yaml. `data` = OccupancyGrid.data (-1 inconnu, 0..100).

    Convention map_server (mode trinary) : libre = 254, occupe = 0, inconnu = 205 ; la
    ligne 0 de la grille est en BAS de l'image, d'ou l'inversion verticale.
    """
    rows = []
    for y in range(height - 1, -1, -1):
        row = bytearray(width)
        for x in range(width):
            v = data[y * width + x]
            if v < 0:
                row[x] = 205
            elif v >= OCC_THRESH * 100:
                row[x] = 0
            elif v <= FREE_THRESH * 100:
                row[x] = 254
            else:
                row[x] = 205
        rows.append(bytes(row))
    with open(base + ".pgm", "wb") as fh:
        fh.write(b"P5\n%d %d\n255\n" % (width, height))
        for r in rows:
            fh.write(r)
    ox, oy, oyaw = origin_xyz_yaw
    meta = {"image": os.path.basename(base) + ".pgm", "mode": "trinary",
            "resolution": float(resolution), "origin": [float(ox), float(oy), float(oyaw)],
            "negate": 0, "occupied_thresh": OCC_THRESH, "free_thresh": FREE_THRESH}
    with open(base + ".yaml", "w", encoding="utf-8") as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)
    return base + ".yaml"


def map_stats(width, height, data):
    """(part connue, part occupee) de la grille, en fractions."""
    n = max(1, width * height)
    known = sum(1 for v in data if v >= 0)
    occ = sum(1 for v in data if v >= OCC_THRESH * 100)
    return known / n, occ / n


def list_maps(maps_dir):
    """Noms des cartes (sans extension), de la plus RECENTE a la plus ancienne."""
    if not os.path.isdir(maps_dir):
        return []
    names = [f[:-5] for f in os.listdir(maps_dir)
             if f.endswith(".yaml") and not f.endswith("_waypoints.yaml")]
    names.sort(key=lambda n: os.path.getmtime(os.path.join(maps_dir, n + ".yaml")),
               reverse=True)
    return names


def load_waypoints(maps_dir, map_name):
    path = os.path.join(maps_dir, map_name + "_waypoints.yaml")
    if not os.path.isfile(path):
        return []
    with open(path, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    return [dict(w) for w in doc.get("waypoints", []) or []]


def save_waypoint(maps_dir, map_name, name, x, y, yaw):
    """Ajoute (ou remplace, a nom egal) un point ; renvoie la liste a jour."""
    wps = [w for w in load_waypoints(maps_dir, map_name) if w.get("name") != name]
    wps.append({"name": name, "x": round(float(x), 3), "y": round(float(y), 3),
                "yaw": round(float(yaw), 4)})
    os.makedirs(maps_dir, exist_ok=True)
    with open(os.path.join(maps_dir, map_name + "_waypoints.yaml"), "w",
              encoding="utf-8") as fh:
        yaml.safe_dump({"waypoints": wps}, fh, sort_keys=False)
    return wps


def save_frontier_log(maps_dir, map_name, entries, home=None):
    """Debriefing de l'exploration automatique : <carte>_frontieres.yaml (numero, position,
    statut de chaque frontiere, et le depart)."""
    path = os.path.join(maps_dir, "%s_frontieres.yaml" % map_name)
    doc = {"depart": ({"x": float(home[0]), "y": float(home[1])} if home else None),
           "frontieres": [{"n": int(e["n"]), "x": float(e["x"]), "y": float(e["y"]),
                           "statut": str(e["s"])} for e in entries]}
    with open(path, "w", encoding="utf-8") as fh:
        yaml.safe_dump(doc, fh, sort_keys=False, allow_unicode=True)
    return path


def yaw_of(qz, qw, qx=0.0, qy=0.0):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
