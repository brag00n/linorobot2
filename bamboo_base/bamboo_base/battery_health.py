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


class ChargerDetector:
    """Branchement / debranchement du CHARGEUR, deduit de la tension AU REPOS.

    Sur BambooWS le chargeur alimente le pack SANS passer par l'INA219 de la carte : le
    courant mesure reste celui du robot (-0,61 A, chargeur branche, 2026-10-12). Reste la
    TENSION : au branchement elle fait un SAUT vers le haut (+0,11 V mesure, 11,91 -> 12,02 V),
    au debranchement un saut vers le bas, et en charge elle MONTE lentement.
    Piege : apres avoir roule, la tension REMONTE seule (relaxation du pack), et ressemble a un
    branchement. On ne compare donc que sur un repos CONTINU d'au moins `quiet_s`.
    """

    def __init__(self, step_v=0.08, quiet_s=60.0, slope_v_min=0.01, slope_span_s=120.0):
        self.step_v = step_v
        self.quiet_s = quiet_s
        self.slope_v_min = slope_v_min
        self.slope_span_s = slope_span_s
        self._hist = collections.deque()         # (t, v) au repos, un point par 0,5 s
        self.charging = None                     # None = inconnu

    def observe(self, t, v, rest_age):
        """`rest_age` = duree du repos continu (s), None en mouvement. Rend None, ou
        ("branche" | "debranche", mesure, "saut V" | "pente V/min") quand l'etat change."""
        if rest_age is None:
            self._hist.clear()
            return None
        h = self._hist
        if h and t - h[-1][0] < 0.5:
            return None
        h.append((t, v))
        while h and t - h[0][0] > max(40.0, self.slope_span_s):
            h.popleft()
        # SAUT : 5 dernieres secondes contre la fenetre [t-30, t-25], repos continu depuis
        # quiet_s AVANT cette fenetre (la relaxation apres conduite est alors finie).
        if rest_age >= self.quiet_s + 30.0:
            rec = [x for u, x in h if t - u <= 5.0]
            past = [x for u, x in h if 25.0 <= t - u <= 30.0]
            if len(rec) >= 3 and len(past) >= 3:
                dv = statistics.median(rec) - statistics.median(past)
                if dv > self.step_v and self.charging is not True:
                    return self._set(True, dv, "saut V")
                if dv < -self.step_v and self.charging is not False:
                    return self._set(False, dv, "saut V")
        # PENTE : chargeur deja branche avant le debut du repos, sans saut vu. Seulement sur un
        # historique COMPLET de slope_span_s : juste apres un branchement la tension redescend
        # un peu (12,01 -> 11,98 V en 30 s, 2026-10-12), et une pente sur ces quelques secondes
        # passait pour un debranchement.
        span = (h[-1][0] - h[0][0]) if h else 0.0
        if rest_age >= self.quiet_s + self.slope_span_s and span >= 0.9 * self.slope_span_s:
            pts = [(u, x) for u, x in h if t - u <= self.slope_span_s]
            if len(pts) >= 10:
                mu = sum(u for u, _ in pts) / len(pts)
                mx = sum(x for _, x in pts) / len(pts)
                den = sum((u - mu) ** 2 for u, _ in pts)
                slope = 60.0 * sum((u - mu) * (x - mx) for u, x in pts) / den if den else 0.0
                if slope > self.slope_v_min and self.charging is not True:
                    return self._set(True, slope, "pente V/min")
                # Jamais de DEBRANCHEMENT sur une pente : un chargeur qui couvre mal la
                # consommation du robot fait lui aussi baisser lentement la tension. Le
                # debranchement se lit sur le SAUT vers le bas (ci-dessus) uniquement.
                # PAS de conclusion sur une tension STABLE : un chargeur qui compense tout juste
                # la consommation du robot (~0,6 A) la laisse stable ALORS QU'IL EST BRANCHE
                # (constate le 2026-10-12 : +0,002 V/min, chargeur branche). Etat inconnu.
        return None

    def _set(self, state, value, kind):
        self.charging = state
        self._hist.clear()                       # le meme saut ne doit pas compter deux fois
        return ("branche" if state else "debranche", value, kind)


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
        self.charger = ChargerDetector()
        self._chgEvents = collections.deque(maxlen=4)

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
        rest_age = (t - self._rest_since) if at_rest and self._rest_since is not None else None
        ev = self.charger.observe(t, v, rest_age)
        if ev:
            self._chgEvents.append(ev)

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

    def pop_charger_event(self):
        """Le prochain evenement chargeur (etat, mesure, nature de la mesure), ou None."""
        return self._chgEvents.popleft() if self._chgEvents else None

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
