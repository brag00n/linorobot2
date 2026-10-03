"""Compensation du GLISSEMENT EN ROTATION d'un chassis a roues fixes (skid-steer).

POURQUOI ICI, DANS LE DRIVER. Sur un 4 roues fixes, tourner fait riper les roues : au sol,
BambooWS a compte 4205 deg de rotation aux roues pour 1972 deg reels a l'IMU (essai du
2026-10-03, rapport 2,13). Une consigne angular.z n'est donc tenue qu'a moitie, et
l'odometrie croit tourner deux fois plus que le robot. Ce defaut vaut quel que soit le
producteur de cmd_vel (manette, navigation, les deux) : la correction se place donc EN AVAL
de tous, au seul endroit par ou ils passent -- le driver de carte. Aucun producteur n'a a
la connaitre.

CE QUI N'EST PAS TOUCHE : l'APB (demi-voie) de la carte. Le firmware ESP32 s'en sert a la
fois pour sa cinematique de consigne ET pour le `wz` qu'il renvoie ; une valeur unique ne
peut pas suivre la vitesse, et le STM32 profite ainsi de la meme correction sans reflasher.

LA CONSTANTE : le taux de glissement en lacet

    s = 1 - w_IMU / w_roues          (0 = aucun glissement ; mesure 2026-10-03 : ~0,53)

    consigne   : w_cmd' = w_cmd / (1 - s)          (feed-forward)
    odometrie  : w_odom = w_roues * (1 - s)

s depend de la vitesse : il est range dans une TABLE |v| x |w| (lignes = v_bins_mps,
colonnes = w_bins_rps), interpolee bilineairement et BORNEE hors de la table. La ligne v=0
vient des rotations sur place de la calibration, les autres des arcs. La table est indexee
sur la vitesse REELLE du corps : c'est elle que demande une consigne, et c'est elle que
l'odometrie cherche (une iteration de point fixe suffit, s variant lentement).

En plus, un PI de VITESSE DE LACET sur le gyro (gz) : w_cmd' += kp*(w_cmd - gz) + ki*int.
Avec w_cmd = 0 et v != 0 le meme PI tient la ligne droite. Il retombe en feed-forward seul
des que gz manque ou date de plus de `gyro_max_age_s`.

TOUT A ZERO = IDENTITE EXACTE (variante `origine`, celle du fichier canonique par defaut) :
meme consigne envoyee a la carte, meme /odom qu'avant ce module. Ce n'est pas "presque" : le
code ne fait aucune operation flottante dans ce cas, et test_motion_compensation le verifie.

OU VIVENT LES CONSTANTES. Dans le fichier canonique du robot, bloc `/**` (donc vues de tous
ses noeuds), une entree par CONFIGURATION MATERIELLE :

    hardware_variant: "origine"
    calib:
      origine: {v_bins_mps: [...], w_bins_rps: [...], yaw_slip: [...], yaw_rate_kp: 0.0, ...}
      caoutchouc_carrelage: {...}

Le coeur (SlipTable, CompensationCore) est du Python pur, testable sans ROS ;
MotionCompensator l'attache aux parametres d'un noeud.
"""

import math

# Cles d'une variante, avec leur valeur NEUTRE. L'ordre est celui du fichier canonique.
VARIANT_KEYS = {
    "description": "",
    "v_bins_mps": [0.0, 0.2, 0.4],
    "w_bins_rps": [0.5, 1.0, 1.5],
    "yaw_slip": [0.0] * 9,
    "yaw_rate_kp": 0.0,
    "yaw_rate_ki": 0.0,
    "yaw_rate_i_max": 0.0,
    # 0 = garder la geometrie du robot ; > 0 = la variante change l'ecartement physique.
    "wheel_separation_m": 0.0,
}

PREFIX = "calib"
DEFAULT_VARIANT = "origine"
# Variante de TRAVAIL, toujours declaree : la calibration y pousse un candidat EN RAM pour
# l'essayer (commande "appliquer en RAM") sans toucher au fichier canonique.
SCRATCH_VARIANT = "essai"


def _bracket(bins, x):
    """(i, j, t) tel que x ~ bins[i]*(1-t) + bins[j]*t, borne aux extremites."""
    n = len(bins)
    if n == 1 or x <= bins[0]:
        return 0, 0, 0.0
    if x >= bins[-1]:
        return n - 1, n - 1, 0.0
    for k in range(n - 1):
        if bins[k] <= x <= bins[k + 1]:
            span = bins[k + 1] - bins[k]
            return k, k + 1, (x - bins[k]) / span if span > 0.0 else 0.0
    return n - 1, n - 1, 0.0


class SlipTable:
    """Table s(|v|, |w|), interpolation bilineaire bornee. Lignes = v, colonnes = w."""

    def __init__(self, v_bins, w_bins, values):
        self.v_bins = [float(x) for x in v_bins]
        self.w_bins = [float(x) for x in w_bins]
        self.values = [float(x) for x in values]
        # Court-circuit de l'identite : aucune multiplication quand tout est nul.
        self.is_zero = all(v == 0.0 for v in self.values)

    @staticmethod
    def check(v_bins, w_bins, values, slip_max):
        """None si valide, sinon la raison (texte du refus de parametre)."""
        if not v_bins or not w_bins:
            return "v_bins_mps et w_bins_rps ne peuvent pas etre vides."
        for name, bins in (("v_bins_mps", v_bins), ("w_bins_rps", w_bins)):
            if any(b < 0.0 for b in bins):
                return "%s : bornes negatives (la table est indexee en valeur absolue)." % name
            if any(bins[k + 1] <= bins[k] for k in range(len(bins) - 1)):
                return "%s doit etre strictement croissant." % name
        if len(values) != len(v_bins) * len(w_bins):
            return ("yaw_slip : %d valeurs pour une table %d x %d (lignes = v, colonnes = w)."
                    % (len(values), len(v_bins), len(w_bins)))
        bad = [x for x in values if not 0.0 <= x <= slip_max]
        if bad:
            return ("yaw_slip hors [0, %.2f] : %s. s = 1 - w_IMU/w_roues, donc 0 = aucun "
                    "glissement ; au-dela de slip_max le gain 1/(1-s) devient dangereux."
                    % (slip_max, bad))
        return None

    def at(self, v_abs, w_abs):
        if self.is_zero:
            return 0.0
        i0, i1, tv = _bracket(self.v_bins, v_abs)
        j0, j1, tw = _bracket(self.w_bins, w_abs)
        nw = len(self.w_bins)
        a = self.values[i0 * nw + j0]
        b = self.values[i0 * nw + j1]
        c = self.values[i1 * nw + j0]
        d = self.values[i1 * nw + j1]
        top = a + (b - a) * tw
        bot = c + (d - c) * tw
        return top + (bot - top) * tv


class CompensationCore:
    """Feed-forward + PI de lacet + odometrie corrigee. Python pur, sans horloge propre.

    `state` garde le dernier calcul (consigne recue, envoyee, s, terme PI) pour
    /motion_compensation/state et le diagnostic.
    """

    def __init__(self, variant=None, slip_max=0.8, max_wheel_mps=0.0,
                 gyro_max_age_s=0.3, wheel_sep=0.0):
        self.slip_max = float(slip_max)
        self.max_wheel_mps = float(max_wheel_mps)
        self.gyro_max_age_s = float(gyro_max_age_s)
        self.wheel_sep = float(wheel_sep)
        self.bypass = False
        self._integ = 0.0
        self._last_t = None
        self.state = {"vx_in": 0.0, "wz_in": 0.0, "vx_out": 0.0, "wz_out": 0.0,
                      "slip": 0.0, "pi": 0.0, "pi_active": False, "gyro_stale": False}
        self.load(variant or dict(VARIANT_KEYS))

    def load(self, variant):
        """Charge une variante DEJA validee (cf. MotionCompensator._check)."""
        self.variant = dict(variant)
        self.table = SlipTable(variant["v_bins_mps"], variant["w_bins_rps"],
                               variant["yaw_slip"])
        self.kp = float(variant["yaw_rate_kp"])
        self.ki = float(variant["yaw_rate_ki"])
        self.i_max = float(variant["yaw_rate_i_max"])
        self._integ = 0.0

    @property
    def pi_enabled(self):
        return self.kp > 0.0 or self.ki > 0.0

    @property
    def is_identity(self):
        return self.table.is_zero and not self.pi_enabled and self.max_wheel_mps <= 0.0

    def reset(self):
        """Arret, watchdog, perte de gyro : l'integrale ne survit pas a une coupure."""
        self._integ = 0.0
        self._last_t = None

    def _gain(self, s):
        return 1.0 / (1.0 - min(s, self.slip_max))

    def command(self, vx, wz, gz=None, gz_age=None, now=None):
        """Consigne de CORPS (celle des producteurs) -> consigne a envoyer a la carte."""
        st = self.state
        st["vx_in"], st["wz_in"] = vx, wz
        st["slip"], st["pi"], st["pi_active"], st["gyro_stale"] = 0.0, 0.0, False, False
        if self.bypass or self.is_identity:
            st["vx_out"], st["wz_out"] = vx, wz
            return vx, wz

        out_w = wz
        if not self.table.is_zero:
            s = self.table.at(abs(vx), abs(wz))
            st["slip"] = s
            out_w = wz * self._gain(s)

        if self.pi_enabled:
            if vx == 0.0 and wz == 0.0:
                self.reset()                 # arret franc : on ne tient rien a l'arret
            elif gz is None or gz_age is None or gz_age > self.gyro_max_age_s:
                st["gyro_stale"] = True
                self.reset()
            else:
                dt = 0.0 if (self._last_t is None or now is None) else max(0.0, now - self._last_t)
                self._last_t = now
                err = wz - gz
                if self.ki > 0.0 and dt > 0.0:
                    self._integ += err * dt
                    if self.i_max > 0.0:
                        self._integ = max(-self.i_max, min(self.i_max, self._integ))
                pi = self.kp * err + self.ki * self._integ
                st["pi"], st["pi_active"] = pi, True
                out_w += pi

        out_v = vx
        if self.max_wheel_mps > 0.0 and self.wheel_sep > 0.0:
            # Saturation des ROUES en gardant le rapport v/w : c'est la consigne que la
            # carte tiendra vraiment (elle sature de meme), et c'est elle qu'on publie.
            half = 0.5 * self.wheel_sep
            peak = max(abs(out_v - out_w * half), abs(out_v + out_w * half))
            if peak > self.max_wheel_mps:
                k = self.max_wheel_mps / peak
                out_v, out_w = out_v * k, out_w * k
        st["vx_out"], st["wz_out"] = out_v, out_w
        return out_v, out_w

    def odometry(self, v, w_wheels):
        """Lacet vu des roues -> lacet REEL estime. Identite si la table est nulle."""
        if self.bypass or self.table.is_zero:
            return w_wheels
        # La table est indexee sur la vitesse REELLE : un pas de point fixe depuis la
        # vitesse roues suffit (s varie lentement avec w).
        s = self.table.at(abs(v), abs(w_wheels))
        s = self.table.at(abs(v), abs(w_wheels) * (1.0 - min(s, self.slip_max)))
        return w_wheels * (1.0 - min(s, self.slip_max))


class MotionCompensator:
    """Attache CompensationCore aux parametres d'un noeud driver.

    Le driver garde la main sur son callback de parametres (Humble n'a qu'un callback
    "pre" utile) : il appelle `check(params)` dans sa phase de validation, puis
    `apply(params)` dans sa phase d'application.

    Parametres declares ici :
      hardware_variant              variante ACTIVE (cle de calib.*)
      calib.<variante>.<cle>        TOUTES les variantes presentes dans le fichier
      motion_compensation_bypass    leve par la calibration le temps de ses mesures
      slip_max, max_wheel_mps, gyro_max_age_s, imu_gz_sign
    """

    def __init__(self, node, wheel_sep):
        self._node = node
        d = node.declare_parameter
        self.variant_name = str(d("hardware_variant", DEFAULT_VARIANT).value)
        slip_max = float(d("slip_max", 0.8).value)
        # 0 = pas de saturation hote (la carte sature de toute facon).
        max_wheel = float(d("max_wheel_mps", 0.0).value)
        gyro_age = float(d("gyro_max_age_s", 0.3).value)
        # Signe de gz par rapport a la convention ROS (z vers le haut, + = anti-horaire).
        # A ETABLIR A L'ESSAI (Tb) : un signe faux ferait DIVERGER le PI au lieu de le
        # corriger -- la calibration le verifie et refuse un signe incoherent.
        self.gz_sign = float(d("imu_gz_sign", 1.0).value)
        bypass = bool(d("motion_compensation_bypass", False).value)

        # Toutes les variantes du fichier, pas seulement l'active : changer de variante a
        # chaud ne doit pas exiger un parametre qu'on n'aurait jamais declare (une
        # surcharge YAML non declaree est invisible en Humble).
        overrides = getattr(node, "_parameter_overrides", {}) or {}
        names = {DEFAULT_VARIANT, SCRATCH_VARIANT, self.variant_name}
        for key in overrides:
            parts = key.split(".")
            if len(parts) == 3 and parts[0] == PREFIX and parts[2] in VARIANT_KEYS:
                names.add(parts[1])
        self.variants = {}
        for name in sorted(names):
            self.variants[name] = {
                k: d("%s.%s.%s" % (PREFIX, name, k), default).value
                for k, default in VARIANT_KEYS.items()}

        self.base_wheel_sep = float(wheel_sep)
        self.core = CompensationCore(slip_max=slip_max, max_wheel_mps=max_wheel,
                                     gyro_max_age_s=gyro_age, wheel_sep=wheel_sep)
        self.core.bypass = bypass
        if self.variant_name not in self.variants:
            raise RuntimeError("hardware_variant '%s' absente de calib.*" % self.variant_name)
        err = self._check_variant(self.variants[self.variant_name], slip_max)
        if err:
            # Au DEMARRAGE, une variante invalide est une erreur de fichier : on le dit et
            # on retombe sur l'identite plutot que de rouler sur des constantes fausses.
            node.get_logger().error("compensation : variante '%s' refusee (%s) -> origine."
                                    % (self.variant_name, err))
            self.variant_name = DEFAULT_VARIANT
        self.core.load(self.variants[self.variant_name])
        self.core.wheel_sep = self.wheel_sep
        node.get_logger().info("compensation de rotation : %s" % self.describe())

    # ------------------------------------------------------------------ lecture
    @property
    def wheel_sep(self):
        """Ecartement en vigueur : celui de la variante s'il est donne, sinon du robot."""
        own = float(self.core.variant.get("wheel_separation_m", 0.0))
        return own if own > 0.0 else self.base_wheel_sep

    def describe(self):
        if self.core.is_identity:
            return "%s (aucune compensation, comportement d'origine)" % self.variant_name
        parts = []
        if not self.core.table.is_zero:
            parts.append("s max %.2f" % max(self.core.table.values))
        if self.core.pi_enabled:
            parts.append("PI kp=%g ki=%g" % (self.core.kp, self.core.ki))
        return "variante %s (%s)" % (self.variant_name, ", ".join(parts) or "saturation seule")

    # ------------------------------------------------------------------ parametres
    @staticmethod
    def _check_variant(v, slip_max):
        err = SlipTable.check([float(x) for x in v["v_bins_mps"]],
                              [float(x) for x in v["w_bins_rps"]],
                              [float(x) for x in v["yaw_slip"]], slip_max)
        if err:
            return err
        for k in ("yaw_rate_kp", "yaw_rate_ki", "yaw_rate_i_max", "wheel_separation_m"):
            if float(v[k]) < 0.0:
                return "%s doit etre >= 0 (0 = inactif)." % k
        return None

    def _merge(self, params):
        """(variantes, active, slip_max, err) apres application HYPOTHETIQUE du lot."""
        variants = {n: dict(v) for n, v in self.variants.items()}
        active = self.variant_name
        slip_max = self.core.slip_max
        for prm in params:
            parts = prm.name.split(".")
            if prm.name == "hardware_variant":
                active = str(prm.value)
            elif prm.name == "slip_max":
                slip_max = float(prm.value)
            elif len(parts) == 3 and parts[0] == PREFIX:
                if parts[1] not in variants:
                    return None, None, None, (
                        "variante '%s' inconnue : la declarer dans le fichier canonique "
                        "(calib.%s.*) puis relancer le driver." % (parts[1], parts[1]))
                if parts[2] not in VARIANT_KEYS:
                    return None, None, None, "cle inconnue : %s" % prm.name
                variants[parts[1]][parts[2]] = prm.value
        if active not in variants:
            return None, None, None, ("hardware_variant '%s' absente de calib.* (connues : %s)."
                                      % (active, ", ".join(sorted(variants))))
        if not 0.0 <= slip_max < 1.0:
            return None, None, None, "slip_max doit etre dans [0, 1[."
        return variants, active, slip_max, None

    def check(self, params):
        """None si le lot est acceptable pour la compensation, sinon la raison."""
        variants, active, slip_max, err = self._merge(params)
        if err:
            return err
        for prm in params:
            if prm.name in ("max_wheel_mps", "gyro_max_age_s") and float(prm.value) < 0.0:
                return "%s doit etre >= 0." % prm.name
            if prm.name == "imu_gz_sign" and float(prm.value) not in (1.0, -1.0):
                return "imu_gz_sign vaut +1 ou -1."
        return self._check_variant(variants[active], slip_max)

    def apply(self, params):
        """Applique un lot DEJA valide par check(). Retourne True si l'ecartement change."""
        before = self.wheel_sep
        variants, active, slip_max, _ = self._merge(params)
        self.variants, self.variant_name = variants, active
        self.core.slip_max = slip_max
        for prm in params:
            if prm.name == "max_wheel_mps":
                self.core.max_wheel_mps = float(prm.value)
            elif prm.name == "gyro_max_age_s":
                self.core.gyro_max_age_s = float(prm.value)
            elif prm.name == "imu_gz_sign":
                self.gz_sign = float(prm.value)
            elif prm.name == "motion_compensation_bypass":
                self.core.bypass = bool(prm.value)
                self.core.reset()
        touched = any(p.name == "hardware_variant" or p.name.startswith(PREFIX + ".")
                      or p.name == "slip_max" for p in params)
        if touched:
            self.core.load(self.variants[self.variant_name])
            self._node.get_logger().info("compensation de rotation : %s" % self.describe())
        self.core.wheel_sep = self.wheel_sep
        return self.wheel_sep != before

    def set_base_wheel_sep(self, sep):
        """Le driver a change wheel_separation_m : nouvelle base si la variante n'en impose pas."""
        self.base_wheel_sep = float(sep)
        self.core.wheel_sep = self.wheel_sep

    # ------------------------------------------------------------------ calcul
    def command(self, vx, wz, gz, gz_age, now):
        g = None if gz is None else self.gz_sign * gz
        return self.core.command(vx, wz, g, gz_age, now)

    def odometry(self, v, w_wheels):
        return self.core.odometry(v, w_wheels)

    def reset(self):
        self.core.reset()

    # ------------------------------------------------------------------ diagnostic
    def diagnose(self, stat, ok, warn):
        """Remplit la tache `compensation` ; ok/warn = niveaux DiagnosticStatus."""
        c = self.core
        stat.add("variante", self.variant_name)
        stat.add("description", str(c.variant.get("description", "")))
        stat.add("table_nulle", str(c.table.is_zero))
        stat.add("s_max_table", "%.3f" % (max(c.table.values) if c.table.values else 0.0))
        stat.add("pi_kp_ki", "%g / %g" % (c.kp, c.ki))
        stat.add("bypass", str(c.bypass))
        stat.add("ecartement_m", "%.4f" % self.wheel_sep)
        stat.add("derniere_consigne", "v %.3f w %.3f -> v %.3f w %.3f (s %.3f, pi %.3f)" % (
            c.state["vx_in"], c.state["wz_in"], c.state["vx_out"], c.state["wz_out"],
            c.state["slip"], c.state["pi"]))
        if c.bypass:
            stat.summary(warn, "compensation NEUTRALISEE (bypass) : normal pendant une "
                               "calibration seulement")
        elif c.pi_enabled and c.state["gyro_stale"]:
            stat.summary(warn, "PI de lacet sans gyro frais : feed-forward seul")
        else:
            stat.summary(ok, self.describe())
        return stat


def body_rate_from_wheels(v_l, v_r, wheel_sep):
    """Lacet vu des roues (rad/s) ; 0 si l'ecartement est inconnu."""
    return (v_r - v_l) / wheel_sep if wheel_sep > 1e-6 else 0.0


def effective_track(wheel_sep, s):
    """Ecartement EFFECTIF equivalent B/(1-s), pour information dans les rapports."""
    return wheel_sep / (1.0 - s) if s < 1.0 else math.inf
