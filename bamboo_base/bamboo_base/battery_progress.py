"""Avancement d'une calibration de batterie, EN DIRECT (Python pur, sans ROS).

Demande utilisateur du 2026-10-18 : calibrer la jauge quelle que soit la qualite du pack, et
montrer pendant l'essai la QUALITE du pack (comparee a un pack de bonne qualite), le pourcentage
estime et la duree restante ; s'arreter seul quand la mesure est suffisante ; ne JAMAIS perdre
les mesures, meme sur une coupure batterie imprevue (le module UPS coupe vers 8,1 V et le RPi
s'eteint sans preavis).

Trois responsabilites, separees pour pouvoir etre testees sans robot :

  BattProgress   les estimations (pourcentage, capacite, duree restante, qualite) et la decision
                 d'arret, alimentees echantillon par echantillon ;
  BattJournal    ce qui survit a une coupure : un journal CSV force sur disque (fsync) a chaque
                 ligne, un fichier d'etat ecrit ATOMIQUEMENT, et le dernier resultat ;
  recover()      au redemarrage, relit le journal d'une session restee `en_cours` et en tire un
                 resultat PARTIEL (jamais utilisable pour la jauge s'il couvre peu de capacite).

Le pack de reference (une « bonne batterie ») est un parametre : 3 x 18650 de 2600 mAh,
50 mohm par element (batt_ref_capacity_ah / batt_ref_r_ohm de calibration.yaml).
"""

import collections
import csv
import json
import math
import os
import statistics
import time

# Courbe TYPIQUE d'un pack en fin de decharge : part de la capacite UTILISABLE consommee quand
# la tension a vide (V + R*I, ramenee a un element) atteint ce niveau. Mesuree sur la decharge
# du 2026-10-17 (pack faible, 0,587 Ah, 55 min), normalisee par pack : le profil d'un pack neuf
# a la meme FORME. Sert UNIQUEMENT a estimer ou on en est avant que les Ah ne parlent.
# (tension a vide par element en V, part consommee 0..1) du depart vers le coude :
FRAC_BY_CELL_V = [(3.887, 0.00), (3.767, 0.11), (3.715, 0.22), (3.673, 0.33), (3.635, 0.44),
                  (3.599, 0.55), (3.559, 0.66), (3.507, 0.77), (3.376, 0.88), (3.103, 1.00)]
# Part de la capacite totale situee SOUS le coude d'arret (9,3 V = 3,1 V par element).
KNEE_TAIL_FRAC = 0.05
# Rien n'est annonce avant ce taux de decharge : la relaxation apres la charge domine.
MIN_USED_FRAC = 0.10


def used_frac_from_cell_v(v_cell, pts=None):
    """Part de la capacite utilisable deja consommee, d'apres la tension a vide d'un element,
    sur la courbe typique du pack (par defaut celle ci-dessus ; chaque type de batterie de
    battery_types.yaml peut porter la sienne). Interpolation lineaire, bornee a 0..1."""
    pts = pts or FRAC_BY_CELL_V
    if v_cell >= pts[0][0]:
        return 0.0
    if v_cell <= pts[-1][0]:
        return 1.0
    for (v0, f0), (v1, f1) in zip(pts, pts[1:]):
        if v1 <= v_cell <= v0:
            return f0 + (f1 - f0) * (v0 - v_cell) / (v0 - v1)
    return None


JOURNAL_COLS = ["epoch", "phase", "v", "i", "ah", "soc", "capacity", "quality", "remaining_s"]


def quality_label(q):
    if q is None:
        return "?"
    return "bon" if q >= 80.0 else ("moyen" if q >= 50.0 else "faible")


def quality_pct(capacity_ah, r_ohm, ref_capacity_ah, ref_r_ohm, sag_ratio=None):
    """Qualite du pack (0..100) par rapport au pack de reference.

    capacite / reference (poids 0,5), R_ref / R (0,3), chute_ref / chute (0,2). Un terme absent
    (resistance non mesuree, pas de chute) reporte son poids sur la capacite, qui reste le
    critere principal : c'est ce que cherche l'utilisateur (« quelle que soit la qualite »)."""
    if capacity_ah is None or capacity_ah <= 0 or ref_capacity_ah <= 0:
        return None
    parts = [(0.5, min(1.0, capacity_ah / ref_capacity_ah))]
    if r_ohm and r_ohm > 0 and ref_r_ohm > 0:
        parts.append((0.3, min(1.0, ref_r_ohm / r_ohm)))
    if sag_ratio is not None and sag_ratio > 0:
        parts.append((0.2, min(1.0, sag_ratio)))
    wsum = sum(w for w, _ in parts)
    return round(100.0 * sum(w * x for w, x in parts) / wsum, 1)


class BattProgress:
    """Estimations et arret d'une decharge. Un appel update() par echantillon (1 Hz)."""

    def __init__(self, cells=3, cutoff_cell_v=3.1, load_cutoff_cell_v=2.9,
                 ref_capacity_ah=2.6, ref_r_ohm=0.15, no_progress_s=1200.0,
                 no_progress_v=0.05, max_duration_s=18000.0, window_s=300.0,
                 method="ah_measured", assumed_current_a=0.0, curve=None, type_name=""):
        self.cells = cells
        # Methode du type de pack : `ah_measured` = on integre le courant mesure (INA219) ;
        # `time_assumed` = pas de mesure (carte STM32), on integre un courant de fond SUPPOSE.
        # Les Ah sont alors une estimation, et la qualite est marquee « estimee ».
        self.method = method
        self.assumed_current_a = assumed_current_a
        self.curve = curve
        self.type_name = type_name
        self.current_assumed = False
        self.cutoff_v = cutoff_cell_v * cells
        self.load_cutoff_v = load_cutoff_cell_v * cells
        self.ref_capacity_ah = ref_capacity_ah
        self.ref_r_ohm = ref_r_ohm
        self.no_progress_s = no_progress_s
        self.no_progress_v = no_progress_v
        self.max_duration_s = max_duration_s
        self.window_s = window_s
        self.reset()

    def reset(self):
        self.t0 = self.t_last = None
        self.ah = 0.0
        self.v_start = None             # tension a vide de depart = point « 100 % »
        self.ocv_start = None
        self.frac_start = 0.0
        self.v, self.i = None, None
        self.r_ohm, self.r_estimated = self.ref_r_ohm, True
        self.sag_ratio = None
        self._i_hist = collections.deque()          # (t, i) sur window_s
        self._cap_hist = collections.deque()        # (t, capacite) pour la stabilite
        self._v_best = None                         # plus basse tension au repos et son instant
        self._t_best = None
        self.stop_reason = ""
        self.capacity_ah = None
        self.soc_pct = None
        self.remaining_s = None
        self.remaining_firm = False
        self.quality = None

    # ------------------------------------------------------------------ entree
    def set_start(self, v_rest, i_a=0.0):
        """Point de depart (le « plein » : charge d'une nuit) mesure par BattRest. C'est le
        100 % PAR CONVENTION (decision utilisateur du 2026-10-18), meme si la tension est bien
        en dessous de 12,6 V : la jauge doit se caler sur ce pack, pas sur un pack theorique."""
        self.v_start = v_rest
        self.ocv_start = v_rest + self.r_ohm * (i_a or 0.0)
        self.frac_start = used_frac_from_cell_v(self.ocv_start / self.cells, self.curve)

    def set_resistance(self, r_ohm, sag_ratio=None):
        """R mesuree sur les cycles (rotation) ; sinon la reference reste, marquee estimee."""
        if r_ohm and 0.02 <= r_ohm <= 2.0:
            self.r_ohm, self.r_estimated = r_ohm, False
        self.sag_ratio = sag_ratio

    def update(self, t, v, i, resting=True):
        """Un echantillon : t (s), v (V), i (A, + = consommation), resting = mesure au repos."""
        if self.t0 is None:
            self.t0 = self.t_last = t
        dt = max(0.0, min(2.0, t - self.t_last))
        self.t_last = t
        if (i is None or i <= 0) and self.method == "time_assumed" and self.assumed_current_a > 0:
            i = self.assumed_current_a          # pas de mesure : courant de fond suppose
            self.current_assumed = True
        self.v, self.i = v, i
        if i is not None and i > 0:
            self.ah += i * dt / 3600.0
            self._i_hist.append((t, i))
        while self._i_hist and t - self._i_hist[0][0] > self.window_s:
            self._i_hist.popleft()
        if resting and v is not None:
            if self._v_best is None or v < self._v_best - self.no_progress_v:
                self._v_best, self._t_best = v, t
        self._estimate(t)
        return self.stop(t, resting)

    # ------------------------------------------------------------------ estimations
    def _ocv(self):
        """Tension a vide : V + R*I sur la derniere mesure."""
        if self.v is None:
            return None
        return self.v + self.r_ohm * (self.i or 0.0)

    def _estimate(self, t):
        ocv = self._ocv()
        f_now = (used_frac_from_cell_v(ocv / self.cells, self.curve)
                 if ocv is not None else None)
        # Part consommee DEPUIS LE DEPART, rapportee a ce qu'il y avait a consommer a partir de
        # la : le depart vaut 100 % quelle que soit sa tension.
        f_start = self.frac_start or 0.0
        used = None
        if f_now is not None and f_now > f_start + 1e-6:
            used = (f_now - f_start) / max(1e-6, 1.0 - f_start)
        # Capacite = Ah consommes / part consommee. Pas avant MIN_USED_FRAC : le debut de la
        # courbe est domine par la relaxation du pack apres la charge.
        cap = None
        if used is not None and used >= MIN_USED_FRAC and self.ah > 0:
            self._cap_hist.append((t, self.ah / used))
            while self._cap_hist and t - self._cap_hist[0][0] > self.window_s:
                self._cap_hist.popleft()
            cap = statistics.median(c for _t, c in self._cap_hist)
        elif self._cap_hist and self.ah > 0:
            # Tension qui remonte un instant (relaxation, charge allegee) : la part consommee
            # repasse sous le seuil. On garde la derniere capacite au lieu de l'effacer.
            cap = statistics.median(c for _t, c in self._cap_hist)
        self.capacity_ah = cap
        # Pourcentage : le compte des Ah des que la capacite est connue (c'est la definition
        # de battery_profile), sinon la part de la courbe typique.
        if cap is not None:
            self.soc_pct = max(0.0, min(100.0, 100.0 * (1.0 - self.ah / cap)))
        elif used is not None:
            self.soc_pct = max(0.0, min(100.0, 100.0 * (1.0 - used)))
        else:
            self.soc_pct = 100.0 if f_now is not None else None
        self.remaining_s = self._remaining()
        self.quality = quality_pct(cap, None if self.r_estimated else self.r_ohm,
                                   self.ref_capacity_ah, self.ref_r_ohm, self.sag_ratio)

    def _remaining(self):
        """Duree restante jusqu'au coude, des que la capacite est estimee (>= 5 echantillons).

        `remaining_firm` dit si l'estimation est stable : ecart entre les centiles 10 et 90 de
        la capacite sur la fenetre <= 15 % de la mediane. Un saut isole (tension qui remonte
        un instant) ne la rend plus « instable » comme l'ancien critere min/max. Sinon la duree
        est PROVISOIRE : donnee quand meme, l'operateur voit un ordre de grandeur."""
        self.remaining_firm = False
        cap = self.capacity_ah
        if cap is None or len(self._cap_hist) < 5:
            return None
        caps = sorted(c for _t, c in self._cap_hist)
        lo = caps[int(0.10 * (len(caps) - 1))]
        hi = caps[int(round(0.90 * (len(caps) - 1)))]
        self.remaining_firm = len(caps) >= 20 and (hi - lo) <= 0.15 * cap
        cur = [x for _t, x in self._i_hist]
        if not cur:
            return None
        i_mean = sum(cur) / len(cur)
        if i_mean < 0.02:
            return None
        left = cap * (1.0 - KNEE_TAIL_FRAC) - self.ah
        return max(0.0, 3600.0 * left / i_mean)

    # ------------------------------------------------------------------ arret
    def stop(self, t, resting=True):
        """Raison d'arret (chaine) ou "" : coude, plafond, ou aucune progression."""
        if self.stop_reason:
            return self.stop_reason
        if self.t0 is not None and t - self.t0 > self.max_duration_s:
            self.stop_reason = "duree maximale %.0f h" % (self.max_duration_s / 3600.0)
        elif resting and self.v is not None and self.v <= self.cutoff_v:
            self.stop_reason = "coude atteint (%.2f V au repos)" % self.v
        elif not resting and self.v is not None and self.v <= self.load_cutoff_v:
            self.stop_reason = "coude atteint sous charge (%.2f V)" % self.v
        elif (self._t_best is not None and t - self._t_best > self.no_progress_s
              and self.t0 is not None and t - self.t0 > self.no_progress_s):
            self.stop_reason = "le pack ne se decharge pas (chargeur branche ?)"
        return self.stop_reason

    # ------------------------------------------------------------------ sortie
    def snapshot(self, phase):
        """Dictionnaire pour BatteryCalibState (NaN = inconnu)."""
        nan = float("nan")
        return {
            "phase": phase,
            "soc_pct": nan if self.soc_pct is None else self.soc_pct,
            "capacity_ah": nan if self.capacity_ah is None else self.capacity_ah,
            "capacity_ref_ah": self.ref_capacity_ah,
            "quality_pct": nan if self.quality is None else self.quality,
            "quality_label": quality_label(self.quality),
            "r_ohm": self.r_ohm, "r_estimated": self.r_estimated or self.current_assumed,
            "pack_type": self.type_name, "current_assumed": self.current_assumed,
            "v_rest": nan if self.v is None else self.v,
            "current_a": nan if self.i is None else self.i,
            "ah_used": self.ah,
            "elapsed_s": 0.0 if self.t0 is None else (self.t_last - self.t0),
            "remaining_s": nan if self.remaining_s is None else self.remaining_s,
            "remaining_firm": self.remaining_firm,
            "stop_reason": self.stop_reason,
        }


def load_battery_types(path):
    """battery_types.yaml -> {nom: dict}. Leve OSError / ValueError si le fichier est illisible
    ou vide : une calibration sans reference de pack n'a pas de sens, on ne devine pas."""
    import yaml
    with open(path, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    types = doc.get("battery_types") or {}
    if not types:
        raise ValueError("%s : aucun bloc battery_types" % path)
    return types


def progress_for_type(types, name, max_duration_s=18000.0, no_progress_s=1200.0,
                      no_progress_v=0.05):
    """BattProgress configure pour le type de pack `name` ; `types` vient de
    load_battery_types. Un nom inconnu ou vide retombe sur le premier type du fichier, avec la
    mention dans type_name, pour qu'un profil de robot sans battery_type reste utilisable."""
    if name not in types:
        name = next(iter(types))
    t = types[name]
    pts = [(float(v), float(f)) for v, f in (t.get("curve") or [])] or None
    return BattProgress(
        cells=int(t["cells"]), cutoff_cell_v=float(t["knee_cell_v"]),
        load_cutoff_cell_v=float(t["load_knee_cell_v"]),
        ref_capacity_ah=float(t["capacity_ah"]), ref_r_ohm=float(t["r_ohm"]),
        no_progress_s=no_progress_s, no_progress_v=no_progress_v,
        max_duration_s=max_duration_s, method=str(t.get("method", "ah_measured")),
        assumed_current_a=float(t.get("assumed_current_a", 0.0)), curve=pts, type_name=name)


# ======================================================================= DONNEES A L'ABRI
def atomic_write_json(path, obj):
    """Ecrit obj en JSON SANS jamais laisser un fichier a moitie ecrit : fichier temporaire
    dans le MEME dossier, fsync, puis os.replace (atomique sur un meme systeme de fichiers)."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=2, ensure_ascii=False, allow_nan=False,
                  default=lambda o: None)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def clean(obj):
    """NaN / inf -> None, recursivement (allow_nan=False refuse sinon)."""
    if isinstance(obj, float) and (math.isnan(obj) or math.isinf(obj)):
        return None
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    return obj


class BattJournal:
    """Journal d'une calibration : survit a une coupure franche.

    batt_journal_<date>.csv   une ligne par seconde, FORCEE sur disque (fsync) : a la coupure,
                              la derniere seconde reste sur la carte SD ;
    batt_session.json         etat, ecrit atomiquement toutes les `state_every_s` secondes ;
    batt_dernier.json         dernier resultat, affiche AVANT le lancement suivant.
    """

    def __init__(self, folder, procedure, state_every_s=10.0, clock=time.time):
        self.folder = folder
        self.procedure = procedure
        self.state_every_s = state_every_s
        self.clock = clock
        os.makedirs(folder, exist_ok=True)
        self.stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(clock()))
        self.path = os.path.join(folder, "batt_journal_%s.csv" % self.stamp)
        self.session_path = os.path.join(folder, "batt_session.json")
        self.last_path = os.path.join(folder, "batt_dernier.json")
        self._fh = open(self.path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._fh)
        self._w.writerow(JOURNAL_COLS)
        self._sync()
        self._t_state = 0.0
        self.session = {"en_cours": True, "procedure": procedure, "journal": self.path,
                        "debut": clock(), "v_depart": None, "cycles": 0}
        atomic_write_json(self.session_path, clean(self.session))

    def _sync(self):
        self._fh.flush()
        os.fsync(self._fh.fileno())

    def log(self, prog, phase):
        snap = prog.snapshot(phase)
        self._w.writerow([
            "%.1f" % self.clock(), phase,
            "" if prog.v is None else "%.3f" % prog.v,
            "" if prog.i is None else "%.3f" % prog.i, "%.5f" % prog.ah,
            "" if prog.soc_pct is None else "%.1f" % prog.soc_pct,
            "" if prog.capacity_ah is None else "%.4f" % prog.capacity_ah,
            "" if prog.quality is None else "%.1f" % prog.quality,
            "" if prog.remaining_s is None else "%.0f" % prog.remaining_s])
        self._sync()
        now = self.clock()
        if now - self._t_state >= self.state_every_s:
            self._t_state = now
            self.session.update({"ah": prog.ah, "capacite_ah": prog.capacity_ah,
                                 "qualite_pct": prog.quality, "phase": phase,
                                 "v_depart": prog.v_start, "maj": now})
            atomic_write_json(self.session_path, clean(self.session))
        return snap

    def set_cycles(self, n):
        self.session["cycles"] = n

    def close(self, prog, reason, partial, extra=None):
        """Fin NORMALE ou annulee : etat `en_cours: false` et dernier resultat."""
        self._sync()
        self._fh.close()
        res = {"date": self.clock(), "procedure": self.procedure, "journal": self.path,
               "capacite_ah": prog.capacity_ah, "qualite_pct": prog.quality,
               "qualite": quality_label(prog.quality), "r_ohm": prog.r_ohm,
               "r_estimee": prog.r_estimated, "ah": prog.ah, "raison_arret": reason,
               "partielle": bool(partial)}
        if extra:
            res.update(extra)
        self.session.update({"en_cours": False, "raison_arret": reason, "maj": self.clock()})
        atomic_write_json(self.last_path, clean(res))
        atomic_write_json(self.session_path, clean(self.session))
        return res


def read_last(folder):
    """Dernier resultat connu (pour l'afficher avant un lancement), ou None."""
    try:
        with open(os.path.join(folder, "batt_dernier.json"), encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def recover(folder, ref_capacity_ah=2.6, ref_r_ohm=0.15, cells=3, min_rows=60):
    """Relit une session restee `en_cours` (coupure imprevue) et en tire un resultat PARTIEL.

    Rend le resultat ecrit dans batt_dernier.json, ou None s'il n'y avait rien a reprendre.
    Le journal est lu en tolerant une derniere ligne tronquee (coupure en pleine ecriture)."""
    sp = os.path.join(folder, "batt_session.json")
    try:
        with open(sp, encoding="utf-8") as fh:
            sess = json.load(fh)
    except (OSError, ValueError):
        return None
    if not sess.get("en_cours"):
        return None
    rows = []
    try:
        with open(sess["journal"], newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                try:
                    rows.append({"v": float(r["v"]), "ah": float(r["ah"]),
                                 "phase": r["phase"], "epoch": float(r["epoch"]),
                                 "cap": float(r["capacity"]) if r["capacity"] else None,
                                 "q": float(r["quality"]) if r["quality"] else None})
                except (ValueError, KeyError, TypeError):
                    continue                       # ligne tronquee par la coupure
    except OSError:
        rows = []
    atomic_write_json(sp, clean(dict(sess, en_cours=False,
                                     raison_arret="interrompue (coupure)")))
    if len(rows) < min_rows:
        return None
    last = rows[-1]
    cap = next((r["cap"] for r in reversed(rows) if r["cap"]), None)
    q = next((r["q"] for r in reversed(rows) if r["q"] is not None), None)
    res = {"date": last["epoch"], "procedure": sess.get("procedure"),
           "journal": sess.get("journal"), "capacite_ah": cap, "qualite_pct": q,
           "qualite": quality_label(q), "r_ohm": ref_r_ohm, "r_estimee": True,
           "ah": last["ah"], "v_derniere": last["v"], "raison_arret": "interrompue (coupure)",
           "partielle": True, "duree_s": last["epoch"] - rows[0]["epoch"]}
    atomic_write_json(os.path.join(folder, "batt_dernier.json"), clean(res))
    return res
