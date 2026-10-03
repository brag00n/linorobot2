"""Diagnostic COMMUN des drivers de carte MAVLink (/diagnostics, lot DIAGNOSTICS puis BASCULE).

Les deux drivers du depot -- stm32_mavlink_driver (bambooSTM32YB) et esp32_mavlink_driver
(BambooWS) -- parlent a leur carte par la MEME couche hote (RobotComSerial et sa sous-classe
Esp32ComSerial), donc disposent des MEMES ingredients de sante : `snapshot()` (ages `*_age`,
compteurs `ok`/`bad`, tension), `connected`, `last_err`, `rx_sysid`, et la relecture de
configuration `board_cfg`. Ce module en tire les CINQ memes taches pour les deux, et c'est ce
qui permet de lire l'arbre de diagnostic d'un robot comme celui de l'autre :

    liaison serie, batterie, imu, configuration carte, actuation

Il ne LIT RIEN sur le port : il travaille sur le dernier snapshot que la boucle de publication
du driver vient de prendre (`driver._snap`, garde par son _tick). Le driver reste le seul
proprietaire du lien.

Ce que le DRIVER fournit (attributs lus a chaque passage, donc commutables a chaud) :
  link, _snap, board_cfg, enable_cmd_vel
  armed_nominal (facultatif)                           l'etat ARME est-il l'etat normal ?
  link_fresh_s, link_error_s, link_bad_warn            seuils de liaison
  batt_warn_v, batt_error_v, batt_over_v               echelle de SON pack
Et, facultatif, `actuation_extra(stat)` pour ce qui n'existe que sur une carte (les servos du
STM32). Une capacite absente n'apparait pas : la tache de l'ESP32 ne parle pas de servos.
"""

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater

_AGES = ("speed_age", "imu_age", "enc_age", "rpm_age", "mag_age")


class DriverDiagnostics:
    """Les cinq taches de diagnostic d'un driver de carte, attachees a son noeud."""

    def __init__(self, driver, hardware_id):
        self._d = driver
        self._ok = self._bad = None
        self.updater = Updater(driver)
        # hardware_id = le SERVICE compose, c'est-a-dire le module de l'arbre de diagnostic.
        self.updater.setHardwareID(hardware_id)
        self.updater.add("liaison serie", self._link)
        self.updater.add("batterie", self._battery)
        self.updater.add("imu", self._imu)
        self.updater.add("configuration carte", self._board_cfg)
        self.updater.add("actuation", self._actuation)

    # ------------------------------------------------------------------ outils
    def link_age(self):
        """Age de la trame la PLUS RECENTE, tous flux confondus ; None si rien n'est arrive.

        C'est la fraicheur de la LIAISON. La batterie s'y rattache : elle arrive aussi par
        une trame `battery` qui ne date rien (RobotComSerial ne pose d'horodatage que sur
        vitesse, attitude, encodeurs, RPM et magneto), donc sa propre fraicheur est inconnue,
        et la tension est la derniere CONNUE sur un lien qui vit.
        """
        ages = [self._d._snap.get(k) for k in _AGES]
        ages = [a for a in ages if a is not None]
        return min(ages) if ages else None

    # ------------------------------------------------------------------ taches
    def _link(self, stat):
        d = self._d
        snap = d._snap
        ok, bad = snap.get("ok"), snap.get("bad")
        d_ok = d_bad = 0
        if ok is not None and self._ok is not None:
            d_ok, d_bad = ok - self._ok, bad - self._bad
        self._ok, self._bad = ok, bad
        age = self.link_age()
        stat.add("port", str(d.link.port))
        stat.add("sysid", str(d.link.rx_sysid))
        stat.add("trames_ok_periode", str(d_ok))
        stat.add("trames_bad_periode", str(d_bad))
        stat.add("trames_ok_total", str(ok))
        stat.add("trames_bad_total", str(bad))
        for k in _AGES:
            a = snap.get(k)
            stat.add(k, "jamais" if a is None else "%.2f" % a)
        if not d.link.connected:
            stat.summary(DiagnosticStatus.ERROR,
                         "port %s non ouvert : %s" % (d.link.port, d.link.last_err))
        elif age is None:
            stat.summary(DiagnosticStatus.ERROR, "port ouvert, aucune trame recue de la carte")
        elif age > d.link_error_s:
            stat.summary(DiagnosticStatus.ERROR, "plus aucune trame depuis %.1f s" % age)
        elif age > d.link_fresh_s:
            stat.summary(DiagnosticStatus.WARN, "trames en retard : %.1f s" % age)
        elif d_bad > d.link_bad_warn:
            stat.summary(DiagnosticStatus.WARN, "%d trames en erreur sur la periode" % d_bad)
        else:
            stat.summary(DiagnosticStatus.OK, "liaison vivante (%d trames/periode)" % d_ok)
        return stat

    def _battery(self, stat):
        d = self._d
        v = d._snap.get("battery")
        age = self.link_age()
        stat.add("seuil_warn_v", "%.1f" % d.batt_warn_v)
        stat.add("seuil_error_v", "%.1f" % d.batt_error_v)
        stat.add("seuil_surtension_v", "%.1f" % d.batt_over_v)
        # PERIMEE, jamais "critique", quand c'est la LIAISON qui manque : /battery publie alors
        # 0 V (snap.get(...) or 0.0) cote STM32, et un diagnostic qui le croirait crierait a la
        # batterie vide au lieu de dire que la carte ne repond plus.
        if v is None or age is None or age > d.link_error_s:
            stat.summary(DiagnosticStatus.STALE, "pas de mesure : liaison avec la carte perdue")
            return stat
        stat.add("tension_v", "%.2f" % v)
        if v >= d.batt_over_v:
            stat.summary(DiagnosticStatus.ERROR, "SURTENSION %.2f V" % v)
        elif v < d.batt_error_v:
            stat.summary(DiagnosticStatus.ERROR, "%.2f V : sous le seuil d'erreur" % v)
        elif v < d.batt_warn_v:
            stat.summary(DiagnosticStatus.WARN, "%.2f V : a surveiller" % v)
        else:
            stat.summary(DiagnosticStatus.OK, "%.2f V" % v)
        return stat

    def _imu(self, stat):
        d = self._d
        a = d._snap.get("imu_age")
        for k in ("roll", "pitch", "yaw"):
            val = d._snap.get(k)
            stat.add(k + "_deg", "?" if val is None else "%.1f" % val)
        stat.add("imu_age", "jamais" if a is None else "%.2f" % a)
        if a is None:
            stat.summary(DiagnosticStatus.ERROR, "aucune trame d'attitude recue")
        elif a > d.link_error_s:
            stat.summary(DiagnosticStatus.ERROR, "attitude figee depuis %.1f s" % a)
        elif a > d.link_fresh_s:
            stat.summary(DiagnosticStatus.WARN, "attitude en retard : %.1f s" % a)
        else:
            stat.summary(DiagnosticStatus.OK, "attitude fraiche")
        return stat

    def _board_cfg(self, stat):
        cfg = self._d.board_cfg
        if cfg is None:
            # WARN et non ERROR : la carte tourne alors sur ce qu'elle a deja (flash pour le
            # STM32, valeurs d'amorcage compilees pour l'ESP32, qui n'a aucune persistance).
            stat.summary(DiagnosticStatus.WARN,
                         "relecture non prouvee : la carte tourne peut-etre sur d'autres valeurs")
            return stat
        stat.add("cpr", "%.0f" % cfg["cpr"])
        stat.add("circ_mm", "%.1f" % cfg["circ_mm"])
        stat.add("apb_mm", "%.1f" % cfg["apb_mm"])
        stat.summary(DiagnosticStatus.OK, "configuration poussee et relue")
        return stat

    def _actuation(self, stat):
        d = self._d
        stat.add("enable_cmd_vel", str(d.enable_cmd_vel))
        extra = getattr(d, "actuation_extra", None)
        if extra is not None:
            extra(stat)
        # L'etat NORMAL depend du robot, et c'est le fichier canonique qui le dit
        # (`armed_nominal`). Hors de cet etat, l'armement est un WARN : pas une panne, mais
        # ce qui doit se voir. bambooSTM32YB : normal = DESARME (pack 12,6 V brut sur moteurs
        # 7,4 V). BambooWS : normal = ARME depuis le 2026-10-03, le seul verrou etant la
        # manette -- un WARN permanent y apprendrait a ignorer le diagnostic.
        nominal = bool(getattr(d, "armed_nominal", False))
        stat.add("arme_est_normal", str(nominal))
        if d.enable_cmd_vel and nominal:
            stat.summary(DiagnosticStatus.OK,
                         "traction armee (etat normal de ce robot : verrou a la manette)")
        elif d.enable_cmd_vel:
            stat.summary(DiagnosticStatus.WARN, "traction ARMEE (enable_cmd_vel=true)")
        else:
            stat.summary(DiagnosticStatus.OK, "traction desarmee (enable_cmd_vel=false)")
        return stat
