"""Calcul de la calibration du glissement en rotation, en Python PUR (teste sans ROS).

Entree : des PALIERS, chacun a consigne (v, w) constante, robot au sol, compensation
NEUTRALISEE (bypass) -- la carte recoit donc la consigne telle quelle. Pour chaque palier,
on a les echantillons du regime etabli (la premiere seconde est exclue par le noeud) :
  gz       vitesse de lacet REELLE (gyro, signee aux conventions ROS)
  w_wheels vitesse de lacet VUE DES ROUES, (v_d - v_g) / ecartement, BRUTE
  v_wheels vitesse lineaire des roues

    s = 1 - mean(gz) / mean(w_wheels)

La TABLE est indexee sur la vitesse REELLE du corps (|v| commande, |w| gyro) -- c'est ce
que motion_compensation.CompensationCore attend : une consigne demande une vitesse REELLE,
l'odometrie la cherche. Comme les paliers sont commandes en vitesse ROUES, leurs vitesses
reelles tombent ou elles tombent (~la moitie de la commande sur BambooWS) : on INTERPOLE
donc s, par ligne de v, sur les bornes w_bins_rps.
"""

import datetime
import json
import math
import statistics


def _mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def rest_stats(gz, yaw_deg, duration_s):
    """Biais, bruit et derive du cap au REPOS. yaw_deg en degres (deroule par l'appelant)."""
    out = {"n": len(gz), "duree_s": duration_s}
    out["biais_gz"] = _mean(gz)
    out["bruit_gz"] = statistics.pstdev(gz) if len(gz) > 1 else float("nan")
    out["derive_cap_deg"] = (yaw_deg[-1] - yaw_deg[0]) if len(yaw_deg) > 1 else float("nan")
    return out


def rest_ok(st, max_bias, max_drift_deg):
    if st["n"] < 5:
        return False, "trop peu d'echantillons gyro au repos (%d)" % st["n"]
    if abs(st["biais_gz"]) > max_bias:
        return False, "biais gyro %.4f rad/s > %.4f" % (st["biais_gz"], max_bias)
    if not math.isnan(st["derive_cap_deg"]) and abs(st["derive_cap_deg"]) > max_drift_deg:
        return False, "derive du cap %.2f deg > %.2f" % (st["derive_cap_deg"], max_drift_deg)
    return True, "biais %.4f rad/s, bruit %.4f, derive %.2f deg" % (
        st["biais_gz"], st["bruit_gz"], st["derive_cap_deg"])


def step_stats(v_cmd, w_cmd, gz, w_wheels, v_wheels):
    """Resume d'un palier ; s = None si les roues ne tournent pas assez pour conclure."""
    g, ww = _mean(gz), _mean(w_wheels)
    st = {"v_cmd": v_cmd, "w_cmd": w_cmd, "n": len(gz),
          "w_imu": g, "w_roues": ww, "v_roues": _mean(v_wheels),
          "dispersion": (statistics.pstdev(gz) / abs(g)) if len(gz) > 1 and g else math.inf}
    st["s"] = (1.0 - g / ww) if abs(ww) > 0.05 else None
    return st


def check_steps(steps, max_dispersion, min_imu_ratio=0.1):
    """Controles de qualite communs. Retourne (fautes bloquantes, avertissements)."""
    faults, warns = [], []
    moving = [s for s in steps if s["s"] is not None]
    if not moving:
        return ["aucun palier exploitable : les roues ne tournent pas (traction coupee ?)"], []
    # Robot SOULEVE : les roues tournent, le gyro ne voit rien.
    lifted = [s for s in moving if abs(s["w_imu"]) < min_imu_ratio * abs(s["w_roues"])]
    if len(lifted) == len(moving):
        faults.append("le gyro ne voit pas tourner le robot alors que les roues tournent : "
                      "robot SOULEVE (roues levees) ?")
        return faults, warns
    # Signe du gyro : roues et gyro doivent tourner dans le MEME sens.
    opposite = [s for s in moving if s["w_imu"] * s["w_roues"] < 0.0]
    if len(opposite) > len(moving) // 2:
        faults.append("gyro et roues tournent en sens OPPOSES sur %d/%d paliers : inverser "
                      "imu_gz_sign dans le fichier canonique" % (len(opposite), len(moving)))
    # Commande et roues dans le meme sens (cablage / inversion d'encodeur).
    wrong = [s for s in moving if s["w_cmd"] * s["w_roues"] < 0.0]
    if wrong:
        faults.append("%d palier(s) ou les roues tournent a l'envers de la consigne" % len(wrong))
    noisy = [s for s in moving if s["dispersion"] > max_dispersion]
    if noisy:
        warns.append("%d palier(s) dispersion gyro > %.0f %%" % (len(noisy), 100 * max_dispersion))
    bad = [s for s in moving if not 0.0 <= s["s"] < 1.0]
    if bad:
        faults.append("s hors [0, 1[ sur %d palier(s) (%s) : mesure incoherente"
                      % (len(bad), ", ".join("%.2f" % s["s"] for s in bad)))
    return faults, warns


def asymmetry(steps, max_asym):
    """Ecart de s entre les deux sens d'un meme |v|,|w_cmd|. Retourne (paires, fautes)."""
    pairs, faults = [], []
    by = {}
    for s in steps:
        if s["s"] is None:
            continue
        by.setdefault((round(abs(s["v_cmd"]), 3), round(abs(s["w_cmd"]), 3)), []).append(s)
    for (v, w), group in sorted(by.items()):
        pos = [g["s"] for g in group if g["w_cmd"] > 0]
        neg = [g["s"] for g in group if g["w_cmd"] < 0]
        if pos and neg:
            sp, sn = _mean(pos), _mean(neg)
            rel = abs(sp - sn) / max(1e-6, 0.5 * (sp + sn))
            pairs.append({"v": v, "w_cmd": w, "s_gauche": sp, "s_droite": sn, "ecart": rel})
            if rel > max_asym:
                faults.append("asymetrie %.0f %% a v=%.2f w=%.2f (s %.3f / %.3f)"
                              % (100 * rel, v, w, sp, sn))
    return pairs, faults


def _interp(points, x):
    """points = [(x, y)] tries ; interpolation lineaire bornee."""
    if not points:
        return None
    if x <= points[0][0]:
        return points[0][1]
    if x >= points[-1][0]:
        return points[-1][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * ((x - x0) / (x1 - x0) if x1 > x0 else 0.0)
    return points[-1][1]


def build_table(steps, v_bins, w_bins, slip_max, fill="copy"):
    """Table s (lignes = v_bins, colonnes = w_bins), indexee sur la vitesse REELLE.

    Une ligne de v sans mesure (arcs non faits) est RECOPIEE de la ligne v=0 (`fill`
    "copy") ou laissee a 0 ("zero"). Retourne (valeurs, lignes_mesurees).
    """
    rows, measured = [], []
    for v in v_bins:
        mine = [s for s in steps if s["s"] is not None
                and abs(abs(s["v_cmd"]) - v) < 0.05]
        pts = sorted((abs(s["w_imu"]), s["s"]) for s in mine)
        # Moyenne des deux sens a vitesse reelle voisine : les points se suivent, l'interp
        # lineaire les lisse assez.
        rows.append([None if not pts else _interp(pts, w) for w in w_bins])
        measured.append(bool(pts))
    base = rows[0] if measured and measured[0] else None
    out = []
    for i, row in enumerate(rows):
        if not measured[i]:
            row = list(base) if (fill == "copy" and base) else [0.0] * len(w_bins)
        out.extend(min(slip_max, max(0.0, x)) for x in row)
    return [round(x, 4) for x in out], measured


def candidate_yaml(variant, robot, table, v_bins, w_bins, measured, report_path, battery_v,
                   wheel_sep):
    """Bloc a RECOPIER dans le fichier canonique, sous `calib:` (apres validation)."""
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    nw = len(w_bins)
    rows = ["[" + ", ".join("%.4f" % x for x in table[i * nw:(i + 1) * nw]) + "]"
            for i in range(len(v_bins))]
    s_mean = sum(table) / len(table) if table else 0.0
    lines = [
        "# CANDIDAT de calibration -- NE PAS charger tel quel : a RECOPIER, apres validation,",
        "# sous `calib:` du fichier canonique de %s (puis hardware_variant: \"%s\")." % (robot,
                                                                                      variant),
        "# Mesure : %s, batterie %.2f V, ecartement physique %.4f m." % (now, battery_v,
                                                                          wheel_sep),
        "# Rapport complet : %s" % report_path,
        "# Lignes mesurees (v_bins) : %s" % ", ".join(
            "%.2f%s" % (v, "" if m else " (RECOPIEE de v=0)") for v, m in zip(v_bins, measured)),
        "# Pour info, ecartement EFFECTIF equivalent B/(1-s) moyen : %.3f m"
        % (wheel_sep / (1.0 - s_mean) if s_mean < 1.0 else float("inf")),
        "      %s:" % variant,
        "        description: \"calibration du %s\"" % now,
        "        v_bins_mps: [%s]" % ", ".join("%.2f" % v for v in v_bins),
        "        w_bins_rps: [%s]" % ", ".join("%.2f" % w for w in w_bins),
        "        yaw_slip: [" + (",\n                   ".join(r[1:-1] for r in rows)) + "]",
        "        yaw_rate_kp: 0.0",
        "        yaw_rate_ki: 0.0",
        "        yaw_rate_i_max: 0.0",
        "        wheel_separation_m: 0.0",
        "",
    ]
    return "\n".join(lines)


def report_json(**kw):
    def clean(x):
        if isinstance(x, float) and (math.isnan(x) or math.isinf(x)):
            return None
        if isinstance(x, dict):
            return {k: clean(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [clean(v) for v in x]
        return x
    return json.dumps(clean(kw), indent=2, ensure_ascii=False)


# ===================================================================== PID des roues
# Reponse d'une roue a un ECHELON de consigne. Entree : temps (s, depuis l'echelon),
# consigne et mesure en RPM telles que le PID de la carte les voit (/req_states,
# /joint_states). La consigne de regime est la mediane de la seconde moitie.

def step_metrics(t, req, meas, rise_frac=0.9):
    """Temps de montee a rise_frac, depassement, erreur statique et oscillation (relatifs)."""
    if len(t) < 5:
        return None
    half = len(req) // 2
    target = statistics.median(req[half:])
    if abs(target) < 1.0:              # sous 1 RPM la mesure ne dit rien
        return None
    y = [m / target for m in meas]     # reponse NORMALISEE, signe absorbe
    rise = next((ti for ti, yi in zip(t, y) if yi >= rise_frac), None)
    tail = y[int(len(y) * 0.7):] or y[-1:]
    over = max(0.0, max(y) - 1.0)
    sse = abs(_mean(tail) - 1.0)
    osc = statistics.pstdev(tail) if len(tail) > 1 else 0.0
    return {"cible_rpm": target, "montee_s": rise, "depassement": over,
            "erreur_statique": sse, "oscillation": osc}


def step_score(m, on_s):
    """Plus petit = meilleur. Une montee jamais atteinte vaut la duree de l'echelon."""
    if m is None:
        return None
    rise = m["montee_s"] if m["montee_s"] is not None else on_s
    return rise + 2.0 * m["depassement"] + 3.0 * m["erreur_statique"] + 2.0 * m["oscillation"]


def best_gains(results, on_s):
    """results : {(kp, ki, kd): [metriques par echelon]} -> (gains, score moyen, table)."""
    table = []
    for gains, steps in results.items():
        scores = [step_score(m, on_s) for m in steps]
        scores = [s for s in scores if s is not None]
        if not scores:
            continue
        table.append((sum(scores) / len(scores), gains, len(scores)))
    table.sort()
    if not table:
        return None, None, []
    return table[0][1], table[0][0], table


def pid_candidate_yaml(robot, gains, score, ref_gains, ref_score, procedure, report_path):
    kp, ki, kd = gains
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    return "\n".join([
        "# CANDIDAT PID des roues -- a RECOPIER, apres validation, a la place de pid_kp/ki/kd",
        "# du fichier canonique de %s (bloc /**)." % robot,
        "# Procedure %s du %s. Score %.3f (gains du fichier %s : %s)."
        % (procedure, now, score, "/".join("%g" % g for g in ref_gains),
           "%.3f" % ref_score if ref_score is not None else "non mesures"),
        "# Rapport complet : %s" % report_path,
        "    pid_kp: %g" % kp,
        "    pid_ki: %g" % ki,
        "    pid_kd: %g" % kd,
        "",
    ])


# ===================================================================== batterie
# Profil de DECHARGE : une ligne par cycle (charge puis repos), avec les Ah consommes
# depuis le debut, la tension au repos en fin de repos, la tension et le courant en fin de
# charge, le courant au repos.

def battery_profile(cycles, cells, n_points=11):
    """-> {capacite_ah, profil_v, profil_soc, r_ohm (par cycle), r_ref_ohm, chute_v}.

    L'etat de charge d'un cycle vaut 1 - Ah_consommes / Ah_total : la decharge s'arrete au
    seuil de coupure, que l'on prend pour ~0 % (un pack ne doit pas descendre plus bas).
    Le profil est reechantillonne sur n_points etats de charge (100, 90, ... 0 %), tension
    de repos DECROISSANTE avec la charge.
    """
    if len(cycles) < 3:
        return None
    total = cycles[-1]["ah"]
    if total <= 0:
        return None
    pts = sorted(((1.0 - c["ah"] / total) * 100.0, c["v_repos"]) for c in cycles)
    # Point du plein : tension au repos AVANT le premier cycle, si mesuree.
    if cycles[0].get("v_repos_initial"):
        pts.append((100.0, cycles[0]["v_repos_initial"]))
        pts.sort()
    soc_axis = [100.0 * k / (n_points - 1) for k in range(n_points)]
    prof = [round(_interp(pts, s), 3) for s in soc_axis]
    r = [round((c["v_repos"] - c["v_charge"]) / (c["i_charge"] - c["i_repos"]), 4)
         for c in cycles if c["i_charge"] - c["i_repos"] > 0.3]
    sag = [c["v_repos"] - c["v_charge"] for c in cycles]
    # R de REFERENCE = mediane sur la moitie haute de la charge (pack sain, charge elevee).
    upper = [rr for c, rr in zip(cycles, r) if 1.0 - c["ah"] / total >= 0.5] or r
    return {"capacite_ah": round(total, 3), "profil_soc": soc_axis, "profil_v": prof,
            "r_ohm": r, "r_ref_ohm": round(statistics.median(upper), 4) if upper else None,
            "chute_v_moy": round(sum(sag) / len(sag), 3), "elements": cells}


def battery_candidate_yaml(robot, prof, full, report_path):
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    r = prof.get("r_ref_ohm")
    lines = [
        "# CANDIDAT batterie -- a RECOPIER, apres validation, dans le bloc /** du fichier",
        "# canonique de %s. Profil de decharge du %s, capacite mesuree %.2f Ah."
        % (robot, now, prof["capacite_ah"]),
        "# Rapport complet : %s" % report_path,
    ]
    if full:
        lines.append("# Plein au repos : %.2f V (%.2f V/element), fond %.2f A."
                     % (full["v"], full["v"] / prof["elements"], full["i"]))
    lines += [
        "    batt_profile_soc: [%s]" % ", ".join("%.0f" % s for s in prof["profil_soc"]),
        "    batt_profile_v: [%s]" % ", ".join("%.3f" % v for v in prof["profil_v"]),
        "    batt_capacity_ah: %.3f" % prof["capacite_ah"],
    ]
    if r:
        # Seuils de WARN derives : +50 % de resistance, 1,5 x la chute moyenne.
        lines += ["    batt_r_ref_ohm: %.4f" % r,
                  "    batt_r_warn_ohm: %.4f" % (1.5 * r),
                  "    batt_sag_warn_v: %.2f" % (1.5 * prof["chute_v_moy"])]
    return "\n".join(lines + [""])
