#!/usr/bin/env python3
"""Lot 4.1 -- T17 : `counts_per_rev` par rotation MANUELLE, roues SURELEVEES.

AUCUNE actuation : ce banc ne publie rien, il ne fait que LIRE les comptages
cumulatifs bruts de `/esp32_status.encoders` (BAMBOO_ENCODERS 42003). Le driver
reste desarme.

Deux questions d'un coup :
  1. les DEUX encodeurs comptent-ils en roue libre ? T16b a vu M1 a 0 tic pendant
     que M2 en comptait 408 -- si l'encodeur gauche ne compte que sous couple,
     tout essai pousse a la main est structurellement faux.
  2. `cpr` vaut-il 2100 ? Delta_tics / 10 tours est exact a un tic pres (~0,05 %),
     sans derivee, sans echelle de temps et SANS GLISSEMENT (roue en l'air).

C'est la mesure qui tranche la troisieme hypothese du +10 % d'odometrie de
T11/T15 : si cpr vaut ~2310, l'odometrie surestime de 10 % sans qu'aucune roue ne
glisse et sans que le diametre soit en cause. Le tachymetre optique avait conclu
~2090, mais en comparant deux grandeurs BRUITEES (RPM optique vs tics/s) avec
+-4 % de dispersion ; ici il n'y a rien a deriver.

On n'auto-detecte RIEN : on journalise la serie temporelle complete, une ligne
par seconde. Les plateaux se lisent a l'oeil, donc un geste parasite -- celui qui
a ruine T16b -- se voit et se contourne au lieu de fausser le resultat.

Voies PHYSIQUES uniquement : index 0 = M1 (gauche), index 1 = M2 (droite).
M3/M4 sont des RECOPIES cote firmware. Si l'index 2 bougeait sans l'index 0, le
mapping serait faux -- c'est un controle gratuit, donc on affiche les quatre.
"""
import sys
import time

import rclpy
from rclpy.node import Node

from bamboo_interfaces.msg import Esp32Status

FENETRE = float(sys.argv[1]) if len(sys.argv) > 1 else 180.0
TOURS = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0


class Lecteur(Node):
    def __init__(self):
        super().__init__("cpr_manual_t17")
        self.enc = None
        self.n = 0
        self.create_subscription(Esp32Status, "/esp32_status", self._s, 20)

    def _s(self, m):
        if len(m.encoders) >= 4:
            self.enc = list(m.encoders)
            self.n += 1


def main():
    rclpy.init()
    n = Lecteur()
    t0 = time.time()
    while n.enc is None and time.time() - t0 < 10.0:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.enc is None:
        print("AUCUN /esp32_status : le driver tourne-t-il ?", flush=True)
        rclpy.shutdown()
        return

    ref = list(n.enc)
    print("ARME. comptages au repos %s" % ref, flush=True)
    print("Tourne la roue avant GAUCHE de %g tours, pause, puis la DROITE de %g "
          "tours." % (TOURS, TOURS), flush=True)
    print("Fenetre %g s. Les colonnes cpr ne valent QUE si %g tours pile ont ete "
          "faits.\n" % (FENETRE, TOURS), flush=True)
    print("   t(s) |   dM1    dM2    dM3    dM4 | cpr M1   cpr M2 | mvt", flush=True)

    prec = list(ref)
    dernier = 0.0
    while time.time() - t0 < FENETRE:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - dernier < 1.0:
            continue
        dernier = time.time()
        cur = list(n.enc)
        d = [a - b for a, b in zip(cur, ref)]
        vit = [a - b for a, b in zip(cur, prec)]
        prec = cur
        # `mvt` nomme la voie qui a bouge dans la derniere seconde : c'est ce qui
        # permet de reperer les plateaux, et de voir un parasite pour ce qu'il est.
        actifs = " ".join("M%d" % (i + 1) for i, v in enumerate(vit) if v != 0)
        print("  %5.1f | %6d %6d %6d %6d | %7.1f %8.1f | %s"
              % (time.time() - t0, d[0], d[1], d[2], d[3],
                 abs(d[0]) / TOURS, abs(d[1]) / TOURS, actifs or "-"), flush=True)

    d = [a - b for a, b in zip(n.enc, ref)]
    print("\n=== T17 : rotation manuelle, %d trames carte ===" % n.n, flush=True)
    print("delta total       : %s tics" % d, flush=True)
    for idx, nom in ((0, "M1 gauche"), (1, "M2 droite")):
        if abs(d[idx]) < 50:
            print("  %s : %+d tic(s) -- AUCUNE rotation vue sur cette voie"
                  % (nom, d[idx]), flush=True)
        else:
            print("  %s : %+d tics -> cpr = %.1f si %g tours (ecart a 2100 : %+.1f %%)"
                  % (nom, d[idx], abs(d[idx]) / TOURS, TOURS,
                     (abs(d[idx]) / TOURS / 2100.0 - 1.0) * 100.0), flush=True)
    print("\nLes plateaux de la serie ci-dessus sont la vraie lecture : le delta "
          "total melange les deux roues.", flush=True)
    rclpy.shutdown()


main()
