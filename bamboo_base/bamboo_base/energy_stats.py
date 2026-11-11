"""Statistiques d'ENERGIE du robot en temps reel et journaux a taille bornee (Python pur, sans ROS).

Demande utilisateur du 2026-11-10 : pouvoir calculer l'autonomie du robot en temps reel pour,
a terme, lui demander d'aller se recharger a un niveau critique, en gardant de quoi rentrer a sa
base. Cette premiere etape est une COLLECTE : elle mesure, elle ne decide rien.

Pourquoi il faut mesurer au lieu de deduire du profil de tension : la decharge de calibration
(2026-10-22) a ete faite robot au repos ou en rotation douce, roues levees (0,66 A, maximum
0,89 A). Le roulage au sol n'a jamais ete mesure, et le pack est fatigue (0,88 Ah pour 2,6 Ah
neufs) : sa tension s'effondre sous charge et sa capacite dépend du courant.

Deux classes, separees pour etre testees sans robot :

  EnergyStats  cumule, echantillon par echantillon, le courant, la puissance et la distance PAR
               ACTIVITE (arret / rotation / ligne / arc), l'energie depuis le debranchement du
               chargeur, la resistance interne en roulage (regression V = V0 - R*I sur une
               fenetre glissante) et une autonomie PROVISOIRE ;
  RotatingLog  journal CSV a 1 Hz dont la taille ne peut PAS grossir sans fin : rotation par
               taille, compression gzip, purge des plus anciens au-dela d'un plafond total, et
               arret des ecritures si le disque est presque plein.

L'autonomie annoncee ici n'est PAS une decision : tant que la capacite observee n'est pas
confirmee par une decharge reelle au sol, `fiable` reste faux.
"""
import collections
import gzip
import json
import math
import os
import shutil
import time

ACTIVITIES = ("arret", "rotation", "ligne", "arc")


def classify(vx, wz, v_stop=0.02, w_stop=0.05, w_arc=0.15):
    """Activite du robot d'apres sa vitesse (m/s) et sa rotation (rad/s) MESUREES."""
    if abs(vx) < v_stop:
        return "arret" if abs(wz) < w_stop else "rotation"
    return "ligne" if abs(wz) < w_arc else "arc"


def _round(x, n):
    return None if x is None or (isinstance(x, float) and not math.isfinite(x)) else round(x, n)


class EnergyStats:
    """Cumuls d'energie. Le courant est compte POSITIF en decharge (le noeud inverse BatteryState)."""

    def __init__(self, capacity_ah=0.88, ema_s=60.0, r_window_s=120.0, r_min_spread_a=0.3,
                 max_gap_s=2.0, reliable_after_s=300.0, impossible_ratio=1.5):
        self.capacity_ah = float(capacity_ah)
        self.ema_s = float(ema_s)
        self.r_window_s = float(r_window_s)
        self.r_min_spread_a = float(r_min_spread_a)
        self.max_gap_s = float(max_gap_s)
        self.reliable_after_s = float(reliable_after_s)
        # Un segment qui consomme plus de impossible_ratio x la capacite du pack SANS que la jauge ait baisse de 5 points est
        # physiquement impossible : le chargeur etait branche sans etre vu (2026-11-10 : 2,69 Ah comptes pour un pack de 0,88 Ah).
        self.impossible_ratio = float(impossible_ratio)
        self.acc = {a: {"t": 0.0, "ah": 0.0, "wh": 0.0, "d": 0.0} for a in ACTIVITIES}
        self._rbuf = collections.deque()
        self._t = None
        self.charging = None
        self.v = self.i = self.soc = None
        self.i_ema = None
        self.activity = "arret"
        self._summary = None
        self._seg = None

    # ---- segment = periode sur batterie, du debranchement du chargeur au rebranchement ----

    def _start_segment(self, t, soc):
        self._seg = {"t0": t, "t": 0.0, "ah": 0.0, "wh": 0.0, "d": 0.0, "soc0": soc,
                     "v0": self.v, "acc0": {a: dict(self.acc[a]) for a in ACTIVITIES}}

    def _implausible(self, s=None):
        """Vrai si le segment depasse la capacite du pack alors que la jauge n'a presque pas bouge."""
        s = s or self._seg
        if s is None or self.capacity_ah <= 0.0 or s["ah"] <= self.impossible_ratio * self.capacity_ah:
            return False
        drop = (s["soc0"] - self.soc) if (s["soc0"] is not None and self.soc is not None) else 0.0
        return drop < 5.0

    def end_segment(self):
        """Clot le segment en cours et en garde le resume (lu par take_summary)."""
        s = self._seg
        self._seg = None
        if s is None or s["t"] < 30.0:
            return
        soc_end = self.soc
        self._summary = {
            "kind": "segment", "invalide": self._implausible(s), "duree_s": round(s["t"], 0), "ah": round(s["ah"], 4),
            "wh": round(s["wh"], 3), "dist_m": round(s["d"], 2),
            "soc_debut": _round(s["soc0"], 1), "soc_fin": _round(soc_end, 1),
            "v_debut": _round(s["v0"], 3), "v_fin": _round(self.v, 3),
            "capacite_obs_ah": _round(self._capacity_obs(s), 3),
            "activites": self._activities(since=s["acc0"]),
            "r_ohm": _round(self._resistance()[0], 3)}

    def take_summary(self):
        s, self._summary = self._summary, None
        return s

    # ---- mise a jour ----

    def update(self, t, v, i, vx, wz, charging, soc=None):
        """Un echantillon (t en s, v en V, i en A positif en decharge, vx m/s, wz rad/s)."""
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        if dt <= 0.0 or dt > self.max_gap_s:
            dt = 0.0
        charging = bool(charging)
        if charging != bool(self.charging):
            if charging:
                self.end_segment()
            else:
                self._rbuf.clear()
                self.v, self.soc = v, soc
                self._start_segment(t, soc)
        first = self.charging is None
        self.charging = charging
        self.v, self.i, self.soc = v, i, soc
        if first and not charging and self._seg is None:
            self._start_segment(t, soc)
        if dt > 0.0:
            a = 1.0 - math.exp(-dt / self.ema_s)
            self.i_ema = i if self.i_ema is None else self.i_ema + a * (i - self.i_ema)
        if charging or dt <= 0.0:
            return
        self.activity = classify(vx, wz)
        d = self.acc[self.activity]
        d["t"] += dt
        d["ah"] += i * dt / 3600.0
        d["wh"] += v * i * dt / 3600.0
        d["d"] += abs(vx) * dt
        s = self._seg
        if s is not None:
            s["t"] += dt
            s["ah"] += i * dt / 3600.0
            s["wh"] += v * i * dt / 3600.0
            s["d"] += abs(vx) * dt
            if s["soc0"] is None and soc is not None:
                s["soc0"] = soc
        self._rbuf.append((t, v, i))
        while self._rbuf and t - self._rbuf[0][0] > self.r_window_s:
            self._rbuf.popleft()

    # ---- grandeurs derivees ----

    def _resistance(self):
        """(R en ohm ou None, nombre d'echantillons). Moindres carres V = V0 - R*I sur la fenetre,
        valable seulement si le courant a assez varie (sinon la pente n'est que du bruit)."""
        buf = self._rbuf
        n = len(buf)
        if n < 30:
            return None, n
        cur = [x[2] for x in buf]
        if max(cur) - min(cur) < self.r_min_spread_a:
            return None, n
        mi = sum(cur) / n
        mv = sum(x[1] for x in buf) / n
        var = sum((c - mi) ** 2 for c in cur)
        if var <= 0.0:
            return None, n
        r = -sum((x[2] - mi) * (x[1] - mv) for x in buf) / var
        return (r if 0.02 <= r <= 1.0 else None), n

    def _capacity_obs(self, s=None):
        """Capacite OBSERVEE : Ah consommes / part de charge perdue, des que la jauge a baisse
        d'au moins 10 points (en dessous, le bruit de la jauge domine)."""
        s = s or self._seg
        if s is None or s["soc0"] is None or self.soc is None:
            return None
        drop = s["soc0"] - self.soc
        return s["ah"] / (drop / 100.0) if drop >= 10.0 and s["ah"] > 0.0 else None

    def _activities(self, since=None):
        out = {}
        for a in ACTIVITIES:
            d = self.acc[a]
            t, ah, wh, dist = d["t"], d["ah"], d["wh"], d["d"]
            if since is not None:
                t, ah, wh, dist = t - since[a]["t"], ah - since[a]["ah"], wh - since[a]["wh"], \
                    dist - since[a]["d"]
            out[a] = {"t_s": round(t, 1), "i_a": _round(ah * 3600.0 / t, 3) if t >= 5.0 else None,
                      "p_w": _round(wh * 3600.0 / t, 2) if t >= 5.0 else None,
                      "dist_m": round(dist, 2)}
        return out

    def snapshot(self):
        """Etat complet, JSON-serialisable (publie a 1 Hz sur /bamboo/energy)."""
        acts = self._activities()
        seg = self._seg
        r, rn = self._resistance()
        d = self.acc["ligne"]
        p_ligne, p_arret = acts["ligne"]["p_w"], acts["arret"]["p_w"]
        wh_m_brut = d["wh"] / d["d"] if d["d"] >= 1.0 else None
        v_ligne = d["d"] / d["t"] if d["t"] >= 5.0 else None
        wh_m_net = ((p_ligne - p_arret) / v_ligne / 3600.0
                    if None not in (p_ligne, p_arret, v_ligne) and v_ligne > 0.01 else None)
        rem = (self.capacity_ah * self.soc / 100.0
               if self.soc is not None and not self.charging else None)

        def minutes(i_a):
            return round(rem / i_a * 60.0, 0) if rem is not None and i_a and i_a > 0.05 else None
        dur = seg["t"] if seg else 0.0
        bad = self._implausible()
        return {
            "charge": bool(self.charging), "v": _round(self.v, 3), "i": _round(self.i, 3),
            "i_ema": _round(self.i_ema, 3), "soc": _round(self.soc, 1),
            "activite": self.activity, "capacite_ah": self.capacity_ah,
            "reste_ah": _round(rem, 3),
            "segment_invalide": bad,
            "autonomie_min": None if bad else minutes(self.i_ema),
            "autonomie_fond_min": minutes(self.acc["arret"]["ah"] * 3600.0 / self.acc["arret"]["t"]
                                          if self.acc["arret"]["t"] >= 60.0 else None),
            "autonomie_roulage_min": minutes(self.acc["ligne"]["ah"] * 3600.0 / d["t"]
                                             if d["t"] >= 20.0 else None),
            "capacite_obs_ah": _round(self._capacity_obs(), 3),
            "fiable": bool(seg and not bad and dur >= self.reliable_after_s and self._capacity_obs()),
            "segment": None if seg is None else {
                "duree_s": round(dur, 0), "ah": round(seg["ah"], 4), "wh": round(seg["wh"], 3),
                "dist_m": round(seg["d"], 2), "soc_debut": _round(seg["soc0"], 1)},
            "activites": acts, "wh_par_m_brut": _round(wh_m_brut, 4),
            "wh_par_m_net": _round(wh_m_net, 4), "r_ohm": _round(r, 3), "r_n": rn}


# =============================================================================================
# Journaux a taille BORNEE
# =============================================================================================

class RotatingLog:
    """Journal CSV qui ne peut pas remplir le disque.

    * rotation : un nouveau fichier horodate tous les `max_bytes` ; l'ancien est compresse
      (gzip, ~8 fois plus petit pour ce CSV) puis le brut est supprime ;
    * purge : au-dela de `max_total` octets pour l'ensemble des fichiers du prefixe, les plus
      anciens sont supprimes (le fichier actif n'est jamais supprime) ;
    * disque : sous `min_free` octets libres, les ecritures sont SUSPENDUES (write rend False),
      verifie toutes les 60 s -- un journal ne doit jamais empecher le robot de travailler ;
    * coupure : fsync au plus toutes les `fsync_s` secondes (la batterie est justement ce qui
      peut couper le RPi sans preavis, mais un fsync par ligne userait la carte SD).
    """

    def __init__(self, directory, prefix, header, max_bytes=2 << 20, max_total=30 << 20,
                 min_free=1 << 30, fsync_s=5.0):
        self.dir, self.prefix, self.header = directory, prefix, list(header)
        self.max_bytes, self.max_total = int(max_bytes), int(max_total)
        self.min_free, self.fsync_s = int(min_free), float(fsync_s)
        self.suspended = False
        self._fh = None
        self._path = None
        self._size = 0
        self._t_fsync = 0.0
        self._t_free = -1e9
        os.makedirs(self.dir, exist_ok=True)
        for p in self._files(raw_only=True):      # reste d'un arret brutal : on le range
            self._compress(p)
        self._purge()

    def _files(self, raw_only=False):
        names = sorted(n for n in os.listdir(self.dir) if n.startswith(self.prefix + "_")
                       and (n.endswith(".csv") if raw_only else n.endswith((".csv", ".csv.gz"))))
        return [os.path.join(self.dir, n) for n in names]

    @staticmethod
    def _compress(path):
        try:
            with open(path, "rb") as src, gzip.open(path + ".gz", "wb", compresslevel=6) as dst:
                shutil.copyfileobj(src, dst)
            os.remove(path)
        except OSError:
            pass

    def _purge(self):
        files = [p for p in self._files() if p != self._path]
        total = sum(os.path.getsize(p) for p in files) + (self._size if self._path else 0)
        while files and total > self.max_total:
            old = files.pop(0)
            try:
                total -= os.path.getsize(old)
                os.remove(old)
            except OSError:
                pass

    def _open(self, now):
        name = "%s_%s.csv" % (self.prefix, time.strftime("%Y%m%d_%H%M%S", time.localtime(now)))
        self._path = os.path.join(self.dir, name)
        self._fh = open(self._path, "a", encoding="utf-8")
        self._size = self._fh.tell()
        if self._size == 0:
            line = ",".join(self.header) + "\n"
            self._fh.write(line)
            self._size = len(line)

    def _rotate(self):
        self.close()
        self._purge()

    def close(self):
        if self._fh is not None:
            try:
                self._fh.flush()
                os.fsync(self._fh.fileno())
                self._fh.close()
            except OSError:
                pass
            self._fh = None
            if self._path:
                self._compress(self._path)
            self._path = None

    def write(self, row, now=None):
        """Ajoute une ligne. Rend False si l'ecriture est suspendue (disque presque plein)."""
        now = time.time() if now is None else now
        if now - self._t_free >= 60.0:
            self._t_free = now
            try:
                self.suspended = shutil.disk_usage(self.dir).free < self.min_free
            except OSError:
                self.suspended = True
        if self.suspended:
            return False
        try:
            if self._fh is None:
                self._open(now)
            line = ",".join("" if x is None else str(x) for x in row) + "\n"
            self._fh.write(line)
            self._fh.flush()
            self._size += len(line)
            if now - self._t_fsync >= self.fsync_s:
                os.fsync(self._fh.fileno())
                self._t_fsync = now
            if self._size >= self.max_bytes:
                self._rotate()
        except OSError:
            self.suspended = True
            return False
        return True


class JsonlTail:
    """Fichier JSON-lignes qui garde seulement ses `keep` dernieres lignes (resumes de segments)."""

    def __init__(self, path, keep=300, max_bytes=256 << 10):
        self.path, self.keep, self.max_bytes = path, int(keep), int(max_bytes)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

    def append(self, obj):
        try:
            with open(self.path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(obj, ensure_ascii=False) + "\n")
                fh.flush()
                os.fsync(fh.fileno())
            if os.path.getsize(self.path) > self.max_bytes:
                with open(self.path, encoding="utf-8") as fh:
                    lines = fh.readlines()[-self.keep:]
                tmp = self.path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as fh:
                    fh.writelines(lines)
                os.replace(tmp, self.path)
        except OSError:
            return False
        return True
