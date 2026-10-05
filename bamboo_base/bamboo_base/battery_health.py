"""Sante de la batterie et son effet sur la TRACTION, estimes en marche (Python pur).

Demande utilisateur du 2026-10-05 : "la batterie donnerait-elle des signes de faiblesse ?
peut-on evaluer son impact sur les perfs et remonter un WARN de diagnostic ?". Une tension
seule ne le dit pas : 11,2 V peut etre un pack a moitie vide au repos, ou un pack plein
qui s'effondre sous charge. Il faut le COURANT (INA219 de l'ESP32, firmware >= 0.5.2) :

  tension au repos (traction arretee depuis rest_settle_s) -> ETAT DE CHARGE (courbe
      Li-ion par element) ; c'est la seule tension qui dise la charge restante ;
  regression V = V0 - R*I sur une fenetre glissante -> RESISTANCE INTERNE R, l'indicateur
      de VIEILLISSEMENT (un pack fatigue a un R qui monte, pas forcement une tension basse) ;
  CHUTE sous charge = V repos - V minimale recente ;
  DECROCHAGE de traction : part du temps de conduite ou une roue reste sous deficit_ratio
      de sa consigne (RPM du PID de la carte) au-dela de deficit_settle_s -- c'est ce qui dit
      si la batterie LIMITE vraiment le robot, et pas seulement qu'elle est basse.

Sans courant (STM32, ESP32 ancien), R et l'etat de repos par courant sont indisponibles :
le repos est alors deduit des consignes nulles, et R reste "non mesure".
"""

import collections
import statistics

# Tension au repos d'UN element Li-ion -> etat de charge (%). Courbe typique NMC/LCO a
# faible courant ; une approximation, pas une jauge coulometrique.
_LIION_CELL = [(3.30, 0), (3.50, 5), (3.60, 12), (3.70, 25), (3.75, 35), (3.80, 45),
               (3.85, 55), (3.90, 62), (3.95, 70), (4.00, 78), (4.10, 90), (4.20, 100)]


def soc_from_rest(v_pack, cells):
    """Etat de charge (%) depuis la tension au REPOS du pack."""
    if v_pack is None or cells <= 0:
        return None
    v = v_pack / cells
    if v <= _LIION_CELL[0][0]:
        return 0.0
    if v >= _LIION_CELL[-1][0]:
        return 100.0
    for (v0, s0), (v1, s1) in zip(_LIION_CELL, _LIION_CELL[1:]):
        if v0 <= v <= v1:
            return s0 + (s1 - s0) * (v - v0) / (v1 - v0)
    return None


class BatteryHealth:

    def __init__(self, cells=3, profile=None, rest_current_a=0.3, rest_settle_s=2.0,
                 window_s=120.0,
                 min_current_spread_a=0.8, deficit_ratio=0.7, deficit_settle_s=1.0,
                 deficit_min_rpm=20.0):
        self.cells = cells
        # Profil MESURE du pack (calibration batterie_decharge) : [(tension repos, %)], qui
        # remplace la courbe Li-ion generique. None = courbe generique.
        self.profile = sorted(profile) if profile and len(profile) >= 2 else None
        self.rest_current_a = rest_current_a
        self.rest_settle_s = rest_settle_s
        self.window_s = window_s
        self.min_spread = min_current_spread_a
        self.deficit_ratio = deficit_ratio
        self.deficit_settle_s = deficit_settle_s
        self.deficit_min_rpm = deficit_min_rpm
        self._vi = collections.deque()           # (t, v, i) pour la regression
        self._rest = collections.deque(maxlen=12)
        self._rest_since = None
        self._low = collections.deque()          # (t, v) sous charge, 10 s
        self._req_since = None
        self._drive = collections.deque()        # (t, dt, deficit, v)
        self._last_t = None
        self.has_current = False

    # ------------------------------------------------------------------ entree
    def observe(self, t, v, i=None, req=None, meas=None):
        """Un echantillon : t (s), v (V), i (A, + en decharge, ou None), RPM demandes et
        mesures par moteur (listes, ou None)."""
        dt = 0.0 if self._last_t is None else max(0.0, min(1.0, t - self._last_t))
        self._last_t = t
        if v is None:
            return
        moving = bool(req) and any(abs(r) > 1.0 for r in req)
        if i is not None:
            self.has_current = True
            self._vi.append((t, v, i))
            while self._vi and t - self._vi[0][0] > self.window_s:
                self._vi.popleft()
        # REPOS = TRACTION A L'ARRET (consignes nulles) depuis rest_settle_s. Pas un seuil de
        # courant : l'electronique seule tire deja ~0,56 A sur BambooWS (mesure 2026-10-05),
        # donc "|I| < 0,3 A" n'etait jamais vrai et l'etat de charge restait inconnu. Le
        # seuil de courant ne sert que sans consignes (carte qui ne les remonte pas).
        if req is not None:
            at_rest = not moving
        else:
            at_rest = i is None or abs(i) < self.rest_current_a
        if at_rest:
            if self._rest_since is None:
                self._rest_since = t
            if t - self._rest_since >= self.rest_settle_s:
                self._rest.append(v)
        else:
            self._rest_since = None
            self._low.append((t, v))
        while self._low and t - self._low[0][0] > 10.0:
            self._low.popleft()

        # Decrochage de traction (M1/M2 : un encodeur par cote sur l'ESP32).
        if req and meas and moving:
            if self._req_since is None:
                self._req_since = t
            settled = t - self._req_since >= self.deficit_settle_s
            pairs = [(r, m) for r, m in zip(req[:2], meas[:2]) if abs(r) >= self.deficit_min_rpm]
            if settled and pairs:
                deficit = any(abs(m) < self.deficit_ratio * abs(r) or m * r < 0 for r, m in pairs)
                self._drive.append((t, dt, deficit, v))
        else:
            self._req_since = None
        while self._drive and t - self._drive[0][0] > self.window_s:
            self._drive.popleft()

    # ------------------------------------------------------------------ estimations
    @property
    def rest_voltage(self):
        return statistics.median(self._rest) if self._rest else None

    @property
    def soc(self):
        v = self.rest_voltage
        if v is None or self.profile is None:
            return soc_from_rest(v, self.cells)
        p = self.profile
        if v <= p[0][0]:
            return float(p[0][1])
        if v >= p[-1][0]:
            return float(p[-1][1])
        for (v0, s0), (v1, s1) in zip(p, p[1:]):
            if v0 <= v <= v1:
                return s0 + (s1 - s0) * (v - v0) / (v1 - v0) if v1 > v0 else float(s1)
        return None

    @property
    def resistance(self):
        """R interne (ohm) par moindres carres V = V0 - R*I, si le courant a assez varie."""
        if len(self._vi) < 10:
            return None
        cur = [x[2] for x in self._vi]
        if max(cur) - min(cur) < self.min_spread:
            return None
        mi = sum(cur) / len(cur)
        mv = sum(x[1] for x in self._vi) / len(self._vi)
        var = sum((c - mi) ** 2 for c in cur)
        cov = sum((x[2] - mi) * (x[1] - mv) for x in self._vi)
        return -cov / var if var > 0 else None

    @property
    def sag(self):
        vr = self.rest_voltage
        if vr is None or not self._low:
            return None
        return max(0.0, vr - min(v for _t, v in self._low))

    @property
    def deficit(self):
        """(part du temps de conduite en decrochage, tension moyenne pendant le
        decrochage, duree de conduite observee)."""
        total = sum(d for _t, d, _x, _v in self._drive)
        if total <= 0:
            return None, None, 0.0
        bad = [(d, v) for _t, d, x, v in self._drive if x]
        frac = sum(d for d, _v in bad) / total
        vbad = (sum(d * v for d, v in bad) / sum(d for d, _v in bad)) if bad else None
        return frac, vbad, total

    def verdict(self, soc_warn_pct, r_warn_ohm, sag_warn_v, deficit_warn_frac,
                min_drive_s=20.0):
        """[] si tout va bien, sinon les raisons du WARN (texte court)."""
        why = []
        s = self.soc
        if s is not None and s < soc_warn_pct:
            why.append("charge ~%.0f %% (repos %.2f V)" % (s, self.rest_voltage))
        r = self.resistance
        if r is not None and r_warn_ohm > 0 and r > r_warn_ohm:
            why.append("resistance interne %.2f ohm > %.2f (pack fatigue ?)" % (r, r_warn_ohm))
        g = self.sag
        if g is not None and sag_warn_v > 0 and g > sag_warn_v:
            why.append("chute sous charge %.2f V" % g)
        frac, vbad, total = self.deficit
        if frac is not None and total >= min_drive_s and frac > deficit_warn_frac:
            why.append("roues sous consigne %.0f %% du temps (a %.2f V)"
                       % (100 * frac, vbad or 0.0))
        return why
