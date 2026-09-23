#!/usr/bin/env python3
"""Lot 4.4 -- T16b : 2 m POUSSES A LA MAIN, bornes detectees par le CAPTEUR.

AUCUNE actuation : ce banc ne publie rien, le driver peut rester desarme
(`enable_cmd_vel false`). C'est l'operateur qui deplace le robot.

Pourquoi les bornes sont detectees et non chronometrees. La premiere version
imposait une fenetre fixe a l'operateur ; T16 est sorti a zero parce que le geste
est tombe hors fenetre. Ici le banc ARME et attend : le depart est le dernier
echantillon AVANT le premier mouvement des comptages, l'arrivee est le premier
echantillon apres STABILITE_S secondes sans le moindre tic. Plus aucune
synchronisation humaine n'est requise.

Ce que l'essai tranche. T11 et T15 ont mesure une odometrie 9,4 % et 10,7 % trop
longue. Sans couple moteur il n'y a PAS de glissement, donc :
  odom ~ ruban       -> c'etait le glissement ; la geometrie est juste.
  odom encore +10 %  -> geometrie ou cpr.

ATTENTION au raisonnement circulaire : `tours = Delta / cpr` suppose le cpr, qui
est justement la valeur suspecte. Le banc affiche donc les TICS BRUTS comme
resultat principal, et le nombre de tours seulement comme lecture SOUS HYPOTHESE.
Seul un temoin independant -- tours comptes a l'oeil sur un repere colle a la
roue, ou les 10 tours de T17 -- permet d'en deduire un cpr.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from bamboo_interfaces.msg import Esp32Status

FENETRE = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
DIAM = float(sys.argv[2]) if len(sys.argv) > 2 else 0.08
CPR = float(sys.argv[3]) if len(sys.argv) > 3 else 2100.0

SEUIL_TICS = 20        # tics cumules au-dela desquels on declare le depart
STABILITE_S = 4.0      # duree sans aucun tic qui declare l'arrivee


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Pousse(Node):
    def __init__(self):
        super().__init__("push_odom_t16b")
        self.xy = None
        self.enc = None
        self.create_subscription(Odometry, "/odom/unfiltered", self._o, 20)
        self.create_subscription(Esp32Status, "/esp32_status", self._s, 20)

    def _o(self, m):
        p = m.pose.pose.position
        self.xy = (p.x, p.y, yaw_de(m.pose.pose.orientation))

    def _s(self, m):
        if len(m.encoders) >= 2:
            self.enc = list(m.encoders)


def bouge(a, b):
    """Amplitude du deplacement entre deux releves de comptage, en tics."""
    return max(abs(x - y) for x, y in zip(a, b))


def ecart(p0, p1):
    """(avance, lateral, rotation deg) dans le repere du DEPART.

    `y - y0` brut ne veut rien dire : x et y vivent dans le repere `odom`, dont le
    cap date du demarrage du driver, pas de cet essai. On projette donc.
    """
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    c, s = math.cos(-p0[2]), math.sin(-p0[2])
    rot = math.degrees(math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2])))
    return dx * c - dy * s, dx * s + dy * c, rot


def main():
    rclpy.init()
    n = Pousse()
    t0 = time.time()
    while (n.xy is None or n.enc is None) and time.time() - t0 < 10.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.xy is None or n.enc is None:
        print("odom=%s encodeurs=%s : le driver tourne-t-il ?"
              % (n.xy is not None, n.enc is not None), flush=True)
        rclpy.shutdown()
        return

    enc_ref, xy_ref = list(n.enc), n.xy          # borne de depart provisoire
    print("ARME. comptages au repos %s | odom (%.4f, %.4f) cap %.2f deg"
          % (enc_ref, xy_ref[0], xy_ref[1], math.degrees(xy_ref[2])), flush=True)
    print("Pousse quand tu veux : le depart est detecte a %d tics." % SEUIL_TICS,
          flush=True)

    t_dep = None
    t_dernier_mvt = None
    enc_fin, xy_fin = None, None
    t_arme = time.time()
    try:
        while time.time() - t_arme < FENETRE:
            rclpy.spin_once(n, timeout_sec=0.1)
            cur = list(n.enc)
            if t_dep is None:
                # tant qu'on n'a pas bouge, la borne de depart SUIT le repos :
                # elle doit etre le dernier etat immobile, pas le premier vu.
                if bouge(cur, enc_ref) < SEUIL_TICS:
                    enc_ref, xy_ref = cur, n.xy
                    continue
                t_dep = time.time()
                t_dernier_mvt = t_dep
                enc_prec = cur
                print("DEPART detecte a t=%.1f s (delta %d tics)"
                      % (t_dep - t_arme, bouge(cur, enc_ref)), flush=True)
                continue

            if bouge(cur, enc_prec) > 0:
                t_dernier_mvt = time.time()
                enc_prec = cur
                enc_fin, xy_fin = cur, n.xy
            elif time.time() - t_dernier_mvt >= STABILITE_S:
                print("ARRIVEE : %.1f s sans un tic." % STABILITE_S, flush=True)
                break
    except KeyboardInterrupt:
        enc_fin, xy_fin = list(n.enc), n.xy
        print("\n(interrompu)", flush=True)

    print("\n=== T16b : pousse a la main, AUCUN couple moteur ===", flush=True)
    if t_dep is None or enc_fin is None:
        print("aucun mouvement detecte pendant %g s." % FENETRE, flush=True)
        rclpy.shutdown()
        return

    d = [a - b for a, b in zip(enc_fin, enc_ref)]
    moy = (abs(d[0]) + abs(d[1])) / 2.0
    av, lat, rot = ecart(xy_ref, xy_fin)
    print("duree poussee     : %.1f s" % (t_dernier_mvt - t_dep), flush=True)
    print("comptages bruts   : %s tics   (M1 %+d, M2 %+d)" % (d, d[0], d[1]), flush=True)
    print("  RESULTAT PRINCIPAL : moyenne des deux voies = %.1f tics" % moy, flush=True)
    print("odom corde        : %.4f m" % math.hypot(xy_fin[0] - xy_ref[0],
                                                    xy_fin[1] - xy_ref[1]), flush=True)
    print("odom avance/lat   : %+.4f / %+.4f m   (rotation %+.2f deg)"
          % (av, lat, rot), flush=True)
    print("\n--- lectures SOUS HYPOTHESE (cpr %.0f, D %.3f m) : a ne pas confondre"
          " avec une mesure ---" % (CPR, DIAM), flush=True)
    print("tours de roue     : %.3f" % (moy / CPR), flush=True)
    print("distance impliquee: %.4f m" % (moy / CPR * math.pi * DIAM), flush=True)
    print("\n--- ce que le RUBAN permettra de calculer, lui sans hypothese ---",
          flush=True)
    print("  cpr reel        = %.1f / (tours COMPTES a l'oeil)" % moy, flush=True)
    print("  circonf. reelle = (distance ruban) / (tours COMPTES a l'oeil)", flush=True)
    print("  tics/m mesure   = %.1f / (distance ruban)   [theorie : %.0f]"
          % (moy, CPR / (math.pi * DIAM)), flush=True)
    rclpy.shutdown()


main()
