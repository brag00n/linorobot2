"""Cartes et points de navigation sur disque (Python pur, format map_server).

DEPUIS 2026-10-12, une carte est un DOSSIER `<maps>/<nom>/` (MapStore) :
  slam.posegraph + slam.data   graphe de poses de slam_toolbox (SerializePoseGraph) : ce qui
                               permet de COMPLETER la carte et de s'y LOCALISER plus tard ;
  map.pgm + map.yaml           l'image (format map_server) : MCP, HUD, relocalisation ;
  meta.yaml                    destinations, gates (zones inconnues), depart, derniere pose ;
  events.jsonl                 l'HISTORIQUE, une ligne JSON par evenement, en ajout seul.
Toute ecriture passe par un fichier temporaire renomme (os.replace) : une coupure de courant
laisse l'ancienne version ou la nouvelle, jamais un fichier a moitie ecrit.

ANCIENNES cartes (avant le 2026-10-12) : <nom>.pgm + <nom>.yaml a plat dans `maps`, points dans
<nom>_waypoints.yaml, debriefing dans <nom>_frontieres.yaml. Sans graphe de poses, elles ne
se completent pas : MapStore les liste comme « ancienne » et sait les IMPORTER en dossier
(image, destinations, gates) pour le MCP et la relocalisation.
L'ecriture de la carte ne passe PAS par nav2_map_saver : on a deja la grille (GetMap), et
l'ecrire soi-meme evite un noeud de plus et un service a attendre.
"""
import datetime
import json
import math
import os
import shutil

import yaml

FREE_THRESH, OCC_THRESH = 0.25, 0.65


def stamp_name(prefix="carte"):
    return "%s_%s" % (prefix, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))


def _atomic_write(path, payload):
    """Ecrit `payload` (bytes ou str) dans `path` via un temporaire renomme."""
    tmp = path + ".tmp"
    with open(tmp, "wb") as fh:
        fh.write(payload if isinstance(payload, bytes) else payload.encode("utf-8"))
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


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
    _atomic_write(base + ".pgm", b"P5\n%d %d\n255\n" % (width, height) + b"".join(rows))
    ox, oy, oyaw = origin_xyz_yaw
    meta = {"image": os.path.basename(base) + ".pgm", "mode": "trinary",
            "resolution": float(resolution), "origin": [float(ox), float(oy), float(oyaw)],
            "negate": 0, "occupied_thresh": OCC_THRESH, "free_thresh": FREE_THRESH}
    _atomic_write(base + ".yaml", yaml.safe_dump(meta, sort_keys=False))
    return base + ".yaml"


def read_pgm(path):
    """(largeur, hauteur, octets) d'une image PGM binaire (P5, 8 bits), lignes de HAUT en bas."""
    with open(path, "rb") as fh:
        raw = fh.read()
    fields, pos = [], 0
    while len(fields) < 4:                     # magique, largeur, hauteur, valeur max
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":           # commentaire jusqu'a la fin de la ligne
            pos = raw.index(b"\n", pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        fields.append(raw[pos:end])
        pos = end
    if fields[0] != b"P5":
        raise ValueError("%s : PGM binaire (P5) attendu, %r lu" % (path, fields[0]))
    w, h = int(fields[1]), int(fields[2])
    return w, h, raw[pos + 1:pos + 1 + w * h]


def read_map(yaml_path):
    """(largeur, hauteur, resolution, (ox, oy, oyaw), data) d'une carte map_server ecrite par
    write_map : `data` au format OccupancyGrid (ligne 0 en BAS ; -1, 0 ou 100)."""
    with open(yaml_path, encoding="utf-8") as fh:
        meta = yaml.safe_load(fh)
    w, h, px = read_pgm(os.path.join(os.path.dirname(yaml_path), meta["image"]))
    data = [-1] * (w * h)
    for row in range(h):
        y = h - 1 - row
        for x in range(w):
            v = px[row * w + x]
            data[y * w + x] = 100 if v < 50 else (0 if v > 250 else -1)
    o = meta.get("origin", [0.0, 0.0, 0.0])
    return w, h, float(meta["resolution"]), (float(o[0]), float(o[1]), float(o[2])), data


def map_stats(width, height, data):
    """(part connue, part occupee) de la grille, en fractions."""
    n = max(1, width * height)
    known = sum(1 for v in data if v >= 0)
    occ = sum(1 for v in data if v >= OCC_THRESH * 100)
    return known / n, occ / n


def list_maps(maps_dir):
    """ANCIENNES cartes a plat (sans extension), de la plus RECENTE a la plus ancienne."""
    if not os.path.isdir(maps_dir):
        return []
    names = [f[:-5] for f in os.listdir(maps_dir)
             if f.endswith(".yaml") and not f.endswith(("_waypoints.yaml", "_frontieres.yaml"))]
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


# ================================================================== MapStore
# Statuts d'une GATE (zone inconnue a explorer, cf. nav_supervisor) :
#   ouverte        a explorer ;
#   en_cours       but de l'exploration en ce moment ;
#   atteinte       le robot y est alle (elle reste parfois au bord de l'inconnu) ;
#   exploree       n'est plus une frontiere : la zone est connue ;
#   abandonnee     Nav2 a echoue, ou le but n'avancait plus ;
#   inatteignable  le planificateur n'y trouve aucun chemin.
GATE_STATES = ("ouverte", "en_cours", "atteinte", "exploree", "abandonnee", "inatteignable")


class MapStore:
    """Les cartes-DOSSIERS de `maps_dir` (cf. en-tete du module) et la carte COURANTE.

    La carte courante est le nom ecrit dans `<maps>/COURANTE` : celle que le superviseur
    reprend au demarrage, que l'exploration complete et sur laquelle la navigation se
    localise. Pur Python (pas de ROS) : teste tel quel.
    """

    CURRENT = "COURANTE"

    def __init__(self, maps_dir):
        self.dir = maps_dir
        os.makedirs(maps_dir, exist_ok=True)

    # ------------------------------------------------------------ chemins
    def path(self, name, fname=""):
        return os.path.join(self.dir, name, fname) if fname else os.path.join(self.dir, name)

    def posegraph_base(self, name):
        """Chemin SANS extension du graphe (slam_toolbox ajoute .posegraph et .data)."""
        return self.path(name, "slam")

    def posegraph_tmp(self, name):
        """Ou faire ecrire le graphe par slam_toolbox, avant commit_posegraph()."""
        return self.path(name, "slam_tmp")

    def has_posegraph(self, name):
        b = self.posegraph_base(name)
        return os.path.isfile(b + ".posegraph") and os.path.isfile(b + ".data")

    def has_image(self, name):
        return os.path.isfile(self.path(name, "map.yaml"))

    def exists(self, name):
        return bool(name) and os.path.isfile(self.path(name, "meta.yaml"))

    # ------------------------------------------------------------ liste et courante
    def list(self):
        """[{name, kind, mtime}], la plus RECENTE d'abord. kind : "carte" (graphe de poses :
        se complete et se charge), "image" (dossier sans graphe), "ancienne" (fichiers a
        plat, a importer)."""
        out = []
        for n in os.listdir(self.dir):
            meta = os.path.join(self.dir, n, "meta.yaml")
            if os.path.isfile(meta):
                out.append({"name": n, "kind": "carte" if self.has_posegraph(n) else "image",
                            "mtime": os.path.getmtime(meta)})
        for n in list_maps(self.dir):
            if not self.exists(n):
                out.append({"name": n, "kind": "ancienne",
                            "mtime": os.path.getmtime(os.path.join(self.dir, n + ".yaml"))})
        out.sort(key=lambda m: m["mtime"], reverse=True)
        return out

    def current(self):
        try:
            with open(os.path.join(self.dir, self.CURRENT), encoding="utf-8") as fh:
                name = fh.read().strip()
        except OSError:
            return ""
        return name if self.exists(name) else ""

    def set_current(self, name):
        _atomic_write(os.path.join(self.dir, self.CURRENT), (name or "") + "\n")

    # ------------------------------------------------------------ creation et import
    def create(self, prefix="carte", name=""):
        """Nouvelle carte VIERGE (dossier + meta) ; rend son nom. Rien n'est jamais ecrase :
        un nom deja pris recoit un suffixe."""
        base = name or stamp_name(prefix)
        name, k = base, 1
        while os.path.exists(self.path(name)):
            k += 1
            name = "%s_%d" % (base, k)
        os.makedirs(self.path(name))
        self.save_meta(name, new_meta(name))
        self.append_event(name, "creation")
        return name

    def import_legacy(self, old):
        """Ancienne carte a plat -> dossier du meme nom (image, destinations, gates). Les
        fichiers d'origine restent en place. Sans graphe de poses : kind "image"."""
        if self.exists(old):
            return old
        y = os.path.join(self.dir, old + ".yaml")
        if not os.path.isfile(y):
            raise FileNotFoundError(y)
        os.makedirs(self.path(old), exist_ok=True)
        w, h, res, origin, data = read_map(y)
        write_map(self.path(old, "map"), w, h, res, origin, data)
        meta = new_meta(old)
        meta["imported_from"] = old
        for wp in load_waypoints(self.dir, old):
            if wp.get("name") == "depart":
                meta["home"] = [float(wp["x"]), float(wp["y"]), float(wp.get("yaw", 0.0))]
            else:
                dest_add(meta, wp["x"], wp["y"], wp.get("yaw", 0.0), wp["name"])
        fl = os.path.join(self.dir, old + "_frontieres.yaml")
        if os.path.isfile(fl):
            with open(fl, encoding="utf-8") as fh:
                doc = yaml.safe_load(fh) or {}
            if doc.get("depart") and meta["home"] is None:
                meta["home"] = [float(doc["depart"]["x"]), float(doc["depart"]["y"]), 0.0]
            for f in doc.get("frontieres") or []:
                st = f.get("statut")
                st = st if st in ("atteinte", "inatteignable", "abandonnee") else (
                    "ouverte" if st == "en_cours" else "abandonnee")
                meta["gates"].append(_gate(int(f["n"]), f["x"], f["y"], 0.0, st))
                meta["gate_seq"] = max(meta["gate_seq"], int(f["n"]))
        self.save_meta(old, meta)
        self.append_event(old, "import", source=old)
        return old

    # ------------------------------------------------------------ meta et historique
    def load_meta(self, name):
        with open(self.path(name, "meta.yaml"), encoding="utf-8") as fh:
            meta = yaml.safe_load(fh) or {}
        base = new_meta(name)
        base.update(meta)
        return base

    def save_meta(self, name, meta):
        meta["updated"] = _now()
        _atomic_write(self.path(name, "meta.yaml"),
                      yaml.safe_dump(meta, sort_keys=False, allow_unicode=True))

    def append_event(self, name, kind, **data):
        """Ajoute une ligne a l'historique. Une seule ecriture en ajout : au pire la DERNIERE
        ligne est tronquee par une coupure, et events() l'ignore."""
        ev = {"t": _now(), "ev": kind}
        ev.update({k: _plain(v) for k, v in data.items()})
        with open(self.path(name, "events.jsonl"), "a", encoding="utf-8") as fh:
            fh.write(json.dumps(ev, ensure_ascii=False) + "\n")
            fh.flush()
            os.fsync(fh.fileno())
        return ev

    def events(self, name, n=50):
        """Les `n` derniers evenements (0 = tous), du plus ancien au plus recent."""
        try:
            with open(self.path(name, "events.jsonl"), encoding="utf-8") as fh:
                lines = fh.readlines()
        except OSError:
            return []
        out = []
        for ln in lines[-n:] if n else lines:
            try:
                out.append(json.loads(ln))
            except ValueError:
                pass                                        # ligne tronquee par une coupure
        return out

    # ------------------------------------------------------------ image et graphe
    def write_grid(self, name, width, height, resolution, origin_xyz_yaw, data):
        return write_map(self.path(name, "map"), width, height, resolution, origin_xyz_yaw,
                         data)

    def backup(self, name, keep=5):
        """Copie le graphe et l'image ACTUELS dans <carte>/historique/<horodatage>/ avant
        qu'une sauvegarde ne les remplace ; garde les `keep` dernieres copies. Essai G1
        (2026-10-13) : apres un saut de localisation, les sauvegardes periodiques avaient
        ECRASE la derniere bonne carte. Rend le dossier cree ("" si rien a copier)."""
        src = [f for f in ("slam.posegraph", "slam.data", "map.pgm", "map.yaml")
               if os.path.isfile(self.path(name, f))]
        if not src:
            return ""
        hist = self.path(name, "historique")
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        dst = os.path.join(hist, stamp)
        os.makedirs(dst, exist_ok=True)
        for f in src:
            shutil.copy2(self.path(name, f), os.path.join(dst, f))
        olds = sorted(d for d in os.listdir(hist) if os.path.isdir(os.path.join(hist, d)))
        for d in olds[:-keep] if keep > 0 else []:
            shutil.rmtree(os.path.join(hist, d), ignore_errors=True)
        return dst

    def history(self, name):
        """Copies de l'historique, de la plus RECENTE a la plus ancienne (chemins)."""
        hist = self.path(name, "historique")
        if not os.path.isdir(hist):
            return []
        return [os.path.join(hist, d) for d in sorted(os.listdir(hist), reverse=True)
                if os.path.isdir(os.path.join(hist, d))]

    def commit_posegraph(self, name):
        """Remplace le graphe par celui que slam_toolbox vient d'ecrire dans posegraph_tmp().
        .data d'abord, .posegraph ensuite : un .posegraph present a toujours ses donnees a
        jour (has_posegraph exige les deux). Rend False si l'ecriture n'a pas eu lieu."""
        tmp, dst = self.posegraph_tmp(name), self.posegraph_base(name)
        if not (os.path.isfile(tmp + ".posegraph") and os.path.isfile(tmp + ".data")):
            return False
        os.replace(tmp + ".data", dst + ".data")
        os.replace(tmp + ".posegraph", dst + ".posegraph")
        return True


def new_meta(name):
    now = _now()
    return {"name": name, "created": now, "updated": now, "home": None, "last_pose": None,
            "dest_seq": 0, "gate_seq": 0, "destinations": [], "gates": []}


# ------------------------------------------------------------ destinations
def _dest(n, name, x, y, yaw):
    return {"n": int(n), "name": str(name), "x": round(float(x), 3), "y": round(float(y), 3),
            "yaw": round(float(yaw), 4)}


def dest_add(meta, x, y, yaw, name=""):
    """Ajoute une destination numerotee (p1, p2... sans nom donne) ; rend la destination.
    Les numeros ne sont jamais reutilises : `p3` supprime, la suivante est `p4`."""
    meta["dest_seq"] = int(meta.get("dest_seq", 0)) + 1
    n = meta["dest_seq"]
    d = _dest(n, name or "p%d" % n, x, y, yaw)
    meta["destinations"].append(d)
    return d


def dest_find(meta, key):
    """Destination par numero ("3" ou 3) ou par nom ; None si absente."""
    key = str(key).strip()
    for d in meta["destinations"]:
        if str(d["n"]) == key or d["name"] == key:
            return d
    return None


def dest_del(meta, key):
    d = dest_find(meta, key)
    if d is not None:
        meta["destinations"].remove(d)
    return d


def dest_rename(meta, key, new):
    d = dest_find(meta, key)
    if d is not None:
        d["name"] = str(new)
    return d


# ------------------------------------------------------------ gates
def _gate(n, x, y, size_m, status="ouverte"):
    return {"n": int(n), "x": round(float(x), 2), "y": round(float(y), 2),
            "size_m": round(float(size_m), 2), "status": status, "tries": 0}


def gates_update(meta, frontiers, match_r=0.5):
    """Apparie les frontieres du moment aux GATES connues (numero stable d'un calcul a
    l'autre) et rend la liste des gates qui sont des frontieres EN CE MOMENT.

    - frontiere a moins de `match_r` d'une gate non exploree : c'est elle (position mise a
      jour : le bord de l'inconnu recule a mesure que la zone se decouvre) ;
    - frontiere sans gate : nouvelle gate `ouverte`, numero suivant ;
    - gate qui n'est plus une frontiere : `exploree` (sauf `en_cours`, dont le but decide).
    """
    live = [g for g in meta["gates"] if g["status"] != "exploree"]
    seen = set()
    for f in sorted(frontiers, key=lambda f: -f.get("size_m", 0.0)):
        best, bd = None, match_r
        for g in live:
            if g["n"] in seen:
                continue
            d = math.hypot(g["x"] - f["x"], g["y"] - f["y"])
            if d < bd:
                best, bd = g, d
        if best is None:
            meta["gate_seq"] = int(meta.get("gate_seq", 0)) + 1
            best = _gate(meta["gate_seq"], f["x"], f["y"], f.get("size_m", 0.0))
            meta["gates"].append(best)
            live.append(best)
        else:
            best["x"], best["y"] = round(float(f["x"]), 2), round(float(f["y"]), 2)
            best["size_m"] = round(float(f.get("size_m", 0.0)), 2)
        seen.add(best["n"])
    for g in live:
        if g["n"] not in seen and g["status"] != "en_cours":
            g["status"] = "exploree"
    return [g for g in meta["gates"] if g["n"] in seen]


def gate_find(meta, n):
    return next((g for g in meta["gates"] if str(g["n"]) == str(n).strip()), None)


def gates_reopen(meta):
    """Nouvelle session d'exploration : les gates en echec (abandonnee, inatteignable) ou
    restees en_cours redeviennent `ouverte` -- une porte fermee hier peut etre ouverte."""
    for g in meta["gates"]:
        if g["status"] in ("abandonnee", "inatteignable", "en_cours"):
            g["status"], g["tries"] = "ouverte", 0


# ------------------------------------------------------------ selection
def cycle(items, current, step):
    """Numero suivant (step +1) ou precedent (-1) dans `items` (dicts a cle "n"), en boucle ;
    None si la liste est vide. `current` absent de la liste : le premier (ou le dernier)."""
    ns = [int(i["n"]) for i in items]
    if not ns:
        return None
    if current not in ns:
        return ns[0] if step > 0 else ns[-1]
    return ns[(ns.index(current) + step) % len(ns)]


def _now():
    return datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")


def _plain(v):
    if isinstance(v, float):
        return round(v, 3)
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    return v
