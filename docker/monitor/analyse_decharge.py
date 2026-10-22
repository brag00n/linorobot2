#!/usr/bin/env python3
"""Analyse d'un historique de decharge (historique_batterie.py) : courbe resumee, premiers
evenements, et NIVEAUX D'ALERTE suggeres.

    python3 analyse_decharge.py decharge_en_cours.csv [--pas 30]

Les niveaux proposes ne sont que des SUGGESTIONS tirees de CE fichier : une decharge sous charge
differente (lidar, Nav2, moteurs) ferait lacher le Wi-Fi plus tot. A confirmer avec un essai en
roulant avant de les ecrire dans les seuils du diagnostic (batt_warn_v, batt_error_v).
"""
import argparse
import csv
import datetime


def num(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("csv")
    ap.add_argument("--pas", type=float, default=30.0, help="minutes entre deux lignes")
    args = ap.parse_args()
    with open(args.csv, newline="", encoding="ascii") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print("fichier vide")
        return
    t0 = num(rows[0], "epoch")
    last = rows[-1]
    with_v = [r for r in rows if num(r, "v_V") is not None]
    print("fichier : %s" % args.csv)
    print("duree   : %.1f h (%d lignes), de %s a %s" % (
        (num(last, "epoch") - t0) / 3600.0, len(rows), rows[0]["heure"], last["heure"]))
    if with_v:
        v0, v1 = num(with_v[0], "v_V"), num(with_v[-1], "v_V")
        print("tension : %.2f V au debut -> %.2f V a la fin" % (v0, v1))

    print("\n   t(h)    V     I(A)   %%     ping(ms) perte30s  Wi-Fi(dBm)  CPU  temp  sous-tension")
    step, nxt = args.pas * 60.0, 0.0
    for r in rows:
        dt = num(r, "epoch") - t0
        if dt < nxt:
            continue
        nxt = dt + step
        print("%7.2f %6s %6s %5s %9s %8s %11s %5s %5s  %s" % (
            dt / 3600.0, r["v_V"][:6], r["i_A"][:5], r["pct"][:5] or "-", r["ping_ms"][:6] or "-",
            r["perte30s_pct"] or "-", r["wifi_dBm"] or "-", r["charge_cpu"] or "-",
            r["temp_C"] or "-", r["sous_tension"] or "-"))

    print("\nPREMIERS EVENEMENTS (tension et niveau au moment, temps avant la derniere ligne) :")
    seen, first = {}, {}
    for r in rows:
        for ev in filter(None, r["evenement"].split(";")):
            key = ev.split(":")[0].split("=")[0]
            seen[key] = seen.get(key, 0) + 1
            first.setdefault(key, (r, ev))
    if not first:
        print("  aucun evenement")
    t_end = num(last, "epoch")
    for key, (r, ev) in sorted(first.items(), key=lambda kv: num(kv[1][0], "epoch")):
        dt = num(r, "epoch") - t0
        print("  %-18s x%-3d a t=%5.2f h  V=%-6s  %%=%-5s  reste %5.1f min jusqu'a la fin" % (
            ev, seen[key], dt / 3600.0, r["v_V"][:6] or "?", r["pct"][:5] or "?",
            (t_end - num(r, "epoch")) / 60.0))

    wl = first.get("wifi_perdu")
    if wl and num(wl[0], "v_V") is not None:
        vw = num(wl[0], "v_V")
        vend = num(with_v[-1], "v_V") if with_v else None
        print("\nNIVEAUX D'ALERTE SUGGERES (a partir du premier Wi-Fi perdu a %.2f V) :" % vw)
        print("  WARN  : %.2f V  (Wi-Fi encore bon, de la marge pour rentrer)" % (vw + 0.30))
        print("  ERROR : %.2f V  (perte du Wi-Fi imminente : ramener le robot a la base)" % (vw + 0.10))
        if vend is not None:
            print("  derniere mesure : %.2f V, soit %.2f V sous le premier Wi-Fi perdu" % (
                vend, vw - vend))
    else:
        print("\nPas de Wi-Fi perdu dans ce fichier : laisser descendre le robot plus bas.")
    print("\nfichier lu le %s" % datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))


if __name__ == "__main__":
    main()
