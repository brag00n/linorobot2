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

    # Seuils releves le 2026-10-13 : en une journee SANS chargeur, 6 faux branchements -- des
    # pentes de +0,010 / +0,011 V/min (relaxation lente d'un pack bas apres avoir roule) et des
    # sauts de 0,08 a 0,19 V (le lidar ou Nav2 qui demarrent : la CHARGE change). Les vrais
    # branchements mesures la veille : sauts de +0,34 / -0,30 / -0,58 V, courant INCHANGE (le
    # chargeur ne passe pas par l'INA219).
    # 2026-10-13 (soir) : chargeur branche PEU APRES avoir gare le robot (saut ignore : repos
    # trop court) puis charge faible (+0,02 V/min) sur une tension bruitee (+-0,02 V) -> jamais
    # vu. D'ou : un GROS saut (big_step_v) compte des 35 s de repos -- la relaxation n'en fait
    # pas autant en quelques secondes ; et la pente, sur 3 min, doit TENIR `slope_hold_s`.
    def __init__(self, step_v=0.10, quiet_s=60.0, slope_v_min=0.015, slope_span_s=180.0,
                 slope_rest_s=360.0, load_di_a=0.08, big_step_v=0.25, slope_hold_s=180.0,
                 reg_min_di_a=0.25, reg_max_slope_v_a=0.05, reg_min_v=11.95,
                 jump_v=0.15, jump_plateau_v=0.04, jump_plateau_s=5.0, jump_di_a=0.12):
        self.step_v = step_v
        self.quiet_s = quiet_s
        self.slope_v_min = slope_v_min
        self.slope_span_s = slope_span_s
        self.slope_rest_s = slope_rest_s
        self.load_di_a = load_di_a
        self.big_step_v = big_step_v
        self.slope_hold_s = slope_hold_s
        # SIGNATURE D'UNE ALIMENTATION REGULEE (2026-10-17) : un pack seul baisse de R*I quand le
        # courant monte (R ~ 0,15-0,3 ohm : 0,15 a 0,3 V par ampere) ; sous un chargeur la tension
        # ne suit pas le courant du robot (<= reg_max_slope_v_a V/A). Il faut un vrai pas de
        # courant (>= reg_min_di_a) pour conclure, et une tension d'au moins reg_min_v.
        self.reg_min_di_a = reg_min_di_a
        self.reg_max_slope_v_a = reg_max_slope_v_a
        self.reg_min_v = reg_min_v
        self._reg = collections.deque(maxlen=600)  # (v, i) en MOUVEMENT ET au repos
        # DETECTION CONTINUE d'un saut (2026-10-17) : sur la tension A TOUT MOMENT, repos ou non
        # (le detecteur par fenetres exige un repos continu et rate un branchement pendant ou juste
        # apres la conduite). Un saut >= jump_v en < 2 s, SUIVI d'un palier (+-jump_plateau_v
        # pendant jump_plateau_s) et sans saut de courant du meme ordre (jump_di_a) : un demarrage
        # de noeud donne un saut de courant, un chargeur non (il ne passe pas par l'INA219).
        self.jump_v = jump_v
        self.jump_plateau_v = jump_plateau_v
        self.jump_plateau_s = jump_plateau_s
        self.jump_di_a = jump_di_a
        self._cont = collections.deque()           # (t, v, i) sur 40 s
        self._pending = None                       # (t0, dv, v_avant, i_avant) a valider
        self._hist = collections.deque()         # (t, v, i) au repos, un point par 0,5 s
        self._slopeSince = None                  # (debut, pente) d'une pente au-dessus du seuil
        self.charging = None                     # None = inconnu

    def _di(self, a, b):
        """Ecart des courants medians de deux tranches ; 0 si le courant n'est pas mesure."""
        ia = [i for _, _, i in a if i is not None]
        ib = [i for _, _, i in b if i is not None]
        if len(ia) < 3 or len(ib) < 3:
            return 0.0
        return statistics.median(ia) - statistics.median(ib)

    @staticmethod
    def _opposes(dv, di, min_di=0.04):
        """Saut de tension et de courant en SENS OPPOSE = changement de CHARGE, pas chargeur.
        Retirer le lidar : le courant baisse (0,61 -> 0,52 A) et la tension MONTE (+0,12 V),
        ce qui passait pour un branchement (2026-10-17). Un chargeur ne passe pas par l'INA219 :
        il laisse le courant du robot a peu pres plat, sans signe privilegie."""
        return (dv > 0 and di < -min_di) or (dv < 0 and di > min_di)

    @staticmethod
    def _slope(pts):
        """Pente (V/min) par moindres carres de [(t, v, i)]."""
        if len(pts) < 5:
            return 0.0
        mu = sum(u for u, _, _ in pts) / len(pts)
        mx = sum(x for _, x, _ in pts) / len(pts)
        den = sum((u - mu) ** 2 for u, _, _ in pts)
        return 60.0 * sum((u - mu) * (x - mx) for u, x, _ in pts) / den if den else 0.0

    def observe_jump(self, t, v, i=None):
        """Saut de tension franc, repos ou non, valide par un palier. Rend l'evenement
        ("branche" | "debranche", dv, "saut V continu") ou None."""
        if v is None:
            return None
        c = self._cont
        c.append((t, v, i))
        while c and t - c[0][0] > 40.0:
            c.popleft()
        if self._pending is None:
            rec = [p for p in c if t - p[0] <= 1.0]
            old = [p for p in c if 6.0 <= t - p[0] <= 12.0]
            if len(rec) < 3 or len(old) < 8:
                return None
            vr = statistics.median(x for _, x, _ in rec)
            vo = statistics.median(x for _, x, _ in old)
            dv = vr - vo
            if abs(dv) < self.jump_v:
                return None
            # La tension d'un pack BAISSE sous charge : un saut vers le bas qui accompagne une
            # hausse du courant est une CHARGE (nœud, moteur), pas un debranchement.
            ir = [x for _, _, x in rec if x is not None]
            io_ = [x for _, _, x in old if x is not None]
            di = (statistics.median(ir) - statistics.median(io_)) if ir and io_ else 0.0
            if abs(di) > self.jump_di_a or self._opposes(dv, di):
                return None
            self._pending = (t, dv, vo, vr)
            return None
        t0, dv, vo, vr = self._pending
        if t - t0 < self.jump_plateau_s:
            return None
        plateau = [x for tt, x, _ in c if tt >= t0]
        self._pending = None
        # Palier : la tension est restee pres du niveau atteint (pas un pic, pas une rampe).
        if max(plateau) - min(plateau) > max(self.jump_plateau_v, 0.25 * abs(dv)) + 0.02 * abs(dv):
            return None
        if dv > 0 and self.charging is not True:
            self._cont.clear()
            return self._set(True, dv, "saut V continu")
        if dv < 0 and self.charging is not False:
            self._cont.clear()
            return self._set(False, dv, "saut V continu")
        return None

    def observe_regulation(self, v, i):
        """Un point (V, I) a toute heure. Rend ("branche", pente V/A, "tension insensible au
        courant") la premiere fois que la tension reste PLATE pour un courant qui a varie."""
        if i is None or v is None or self.charging is not None:
            return None
        self._reg.append((v, i))
        if len(self._reg) < 120:
            return None
        cur = [c for _v, c in self._reg]
        di = max(cur) - min(cur)
        if di < self.reg_min_di_a or statistics.median(x for x, _c in self._reg) < self.reg_min_v:
            return None
        mi = sum(cur) / len(cur)
        mv = sum(x for x, _c in self._reg) / len(self._reg)
        var = sum((c - mi) ** 2 for c in cur)
        slope = sum((c - mi) * (x - mv) for x, c in self._reg) / var if var else 0.0
        # slope = dV/dI : ~ -R sur un pack (negatif, 0,15 a 0,3), ~0 sous un chargeur regule.
        if -self.reg_max_slope_v_a <= slope <= self.reg_max_slope_v_a:
            return self._set(True, slope, "tension insensible au courant")
        return None

    def observe(self, t, v, rest_age, i=None):
        """`rest_age` = duree du repos continu (s), None en mouvement ; `i` = courant du robot
        (A, INA219), None si non mesure. Rend None, ou ("branche" | "debranche", mesure,
        "saut V" | "pente V/min") quand l'etat change."""
        if rest_age is None:
            self._hist.clear()
            self._slopeSince = None
            return None
        h = self._hist
        if h and t - h[-1][0] < 0.5:
            return None
        h.append((t, v, i))
        while h and t - h[0][0] > max(40.0, self.slope_span_s):
            h.popleft()
        # SAUT : 5 dernieres secondes contre la fenetre [t-30, t-25], repos continu depuis
        # quiet_s AVANT cette fenetre (la relaxation apres conduite est alors finie). Un saut
        # ACCOMPAGNE d'un saut de courant est un changement de CHARGE (lidar, Nav2...), pas le
        # chargeur, qui ne passe pas par l'INA219.
        if rest_age >= 35.0:
            # Seuil ordinaire apres quiet_s de repos ; avant, seul un GROS saut compte.
            need = self.step_v if rest_age >= self.quiet_s + 30.0 else self.big_step_v
            rec = [p for p in h if t - p[0] <= 5.0]
            past = [p for p in h if 25.0 <= t - p[0] <= 30.0]
            di_w = self._di(rec, past) if len(rec) >= 3 and len(past) >= 3 else 0.0
            if len(rec) >= 3 and len(past) >= 3 and abs(di_w) <= self.load_di_a:
                dv = statistics.median(x for _, x, _ in rec) - \
                    statistics.median(x for _, x, _ in past)
                if self._opposes(dv, di_w):
                    return None
                if dv > need and self.charging is not True:
                    return self._set(True, dv, "saut V")
                if dv < -need and self.charging is not False:
                    return self._set(False, dv, "saut V")
        # PENTE : chargeur deja branche avant le debut du repos, sans saut vu. Seulement sur un
        # historique COMPLET de slope_span_s : juste apres un branchement la tension redescend
        # un peu (12,01 -> 11,98 V en 30 s, 2026-10-12), et une pente sur ces quelques secondes
        # passait pour un debranchement. Et seulement apres `slope_rest_s` de repos, a courant
        # stable, sur une pente qui TIENT : au-dessus du seuil a chaque mesure pendant
        # `slope_hold_s`, et encore a 60 % au moins de sa valeur du debut. La RELAXATION d'un
        # pack, elle, ralentit sans cesse. (Le test « moitie recente contre moitie ancienne »
        # d'une seule fenetre echouait sur une tension bruitee, 2026-10-13.)
        span = (h[-1][0] - h[0][0]) if h else 0.0
        if rest_age >= self.slope_rest_s and span >= 0.9 * self.slope_span_s:
            pts = [p for p in h if t - p[0] <= self.slope_span_s]
            half = t - self.slope_span_s / 2.0
            old = [p for p in pts if p[0] < half]
            new = [p for p in pts if p[0] >= half]
            slope = self._slope(pts)
            if slope > self.slope_v_min and abs(self._di(new, old)) <= self.load_di_a:
                if self._slopeSince is None:
                    self._slopeSince = (t, slope)
                elif t - self._slopeSince[0] >= self.slope_hold_s and \
                        slope >= 0.6 * self._slopeSince[1] and self.charging is not True:
                    return self._set(True, slope, "pente V/min")
            else:
                self._slopeSince = None
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
        self._slopeSince = None
        return ("branche" if state else "debranche", value, kind)


class BatteryHealth:

    def __init__(self, cells=3, profile=None, rest_current_a=0.3, rest_settle_s=2.0,
                 window_s=120.0,
                 min_current_spread_a=0.8, deficit_ratio=0.7, deficit_settle_s=1.0,
                 deficit_min_rpm=20.0, r_default_ohm=0.20, slew_pct_min=1.0,
                 rest_current_spread_a=0.30, rest_stable_s=10.0,
                 ocv_max_age_s=180.0):
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
        # JAUGE (2026-10-17) : resistance interne supposee tant qu'elle n'est pas mesuree, pente
        # maximale de l'affichage (points de % par minute), et critere de courant STABLE du repos.
        self.r_default_ohm = r_default_ohm
        self.slew_pct_min = slew_pct_min
        self.rest_current_spread_a = rest_current_spread_a
        self.rest_stable_s = rest_stable_s
        self.ocv_max_age_s = ocv_max_age_s
        self._ocv = collections.deque(maxlen=12)  # (t, V + R*I) a vide estimee, repos stable
        self._shown = None                        # (t, %) dernier pourcentage affiche
        self._iRest = collections.deque()         # (t, i) pour juger la stabilite du courant
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
        if i is not None:
            self._iRest.append((t, i))
            while self._iRest and t - self._iRest[0][0] > self.rest_stable_s:
                self._iRest.popleft()
        if at_rest:
            if self._rest_since is None:
                self._rest_since = t
            if t - self._rest_since >= self.rest_settle_s:
                self._rest.append(v)
                # Tension A VIDE du pack : on ajoute la chute interne R*I (0,1 a 0,2 V avec le
                # robot qui tire ~0,6 A en permanence), mais SEULEMENT sur un courant stable --
                # un echantillon pris pendant un demarrage de noeud serait faux.
                if self._current_stable():
                    r = self.resistance
                    r = self.r_default_ohm if r is None or not 0.02 <= r <= 1.0 else r
                    self._ocv.append((t, v + r * (i or 0.0)))
        else:
            self._rest_since = None
            self._low.append((t, v))
        while self._low and t - self._low[0][0] > 10.0:
            self._low.popleft()
        rest_age = (t - self._rest_since) if at_rest and self._rest_since is not None else None
        ev = (self.charger.observe(t, v, rest_age, i) or self.charger.observe_jump(t, v, i)
              or self.charger.observe_regulation(v, i))
        if ev:
            self._chgEvents.append(ev)
            # La tension d'AVANT l'evenement n'est plus celle du pack (2026-10-17 : 85 % affiches
            # pour 50 % reels, des valeurs de la periode branchee restees en memoire).
            self._ocv.clear()
            self._rest.clear()
            self._shown = None

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

    def _current_stable(self):
        """Courant plat sur rest_stable_s (ecart < rest_current_spread_a) ; vrai sans mesure."""
        if not self.has_current:
            return True
        cur = [c for _t, c in self._iRest]
        if len(cur) < 5 or self._iRest[-1][0] - self._iRest[0][0] < 0.8 * self.rest_stable_s:
            return False
        return max(cur) - min(cur) < self.rest_current_spread_a

    @property
    def ocv(self):
        """Tension a vide estimee (V), ou la tension de repos brute sans mesure de courant."""
        fresh = [x for tt, x in self._ocv
                 if self._last_t is not None and self._last_t - tt <= self.ocv_max_age_s]
        return statistics.median(fresh) if fresh else self.rest_voltage

    @property
    def r_used(self):
        r = self.resistance
        return self.r_default_ohm if r is None or not 0.02 <= r <= 1.0 else r

    def _soc_at(self, v):
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
    def soc_raw(self):
        """Etat de charge instantane, sur la tension A VIDE compensee. None sans repos stable."""
        return self._soc_at(self.ocv)

    def display_soc(self, t):
        """Pourcentage A AFFICHER (HUD, /battery) : None tant que le chargeur est branche ou
        possible (la tension est alors celle du CHARGEUR, pas du pack -- 83 % affiches branche,
        61 % debranche, 2026-10-17), sinon l'etat de charge lisse. Il DESCEND a pente limitee et
        ne REMONTE PAS sans chargeur confirme : un pack ne se recharge pas seul, ses remontees
        sont de la relaxation ou du bruit. Le premier calcul n'est pas limite."""
        if self.charger.charging:
            self._shown = None               # au debranchement, on repart de la mesure
            return None
        raw = self.soc_raw
        if raw is None:
            return None if self._shown is None else self._shown[1]
        if self._shown is None or abs(raw - self._shown[1]) > 8.0:
            # Demarrage, ou ecart de plus de 8 points avec l'affichage (dans les deux sens) :
            # ce n'est plus de la relaxation ni du bruit, la jauge s'aligne tout de suite sur la
            # mesure. Le lissage ne sert qu'aux petites variations (2026-10-17 : 76 % affiches
            # pour 48 % mesures, 30 min de descente avant de rattraper).
            self._shown = (t, raw)
            return raw
        t0, last = self._shown
        step = self.slew_pct_min * max(0.0, t - t0) / 60.0
        shown = max(raw, last - step) if raw < last else last
        self._shown = (t, shown)
        return shown

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
