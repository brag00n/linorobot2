"""incident_model.py : de l'arbre de diagnostic (/diagnostics_agg) aux INCIDENTS. Python PUR, sans ROS : teste hors du robot.

Chaine : taches feuilles en defaut -> PARTS en defaut (via parts.yaml) -> racines et impact (parts_model.Graph) -> incidents suivis dans le temps -> JSON.

Regles pour ne PAS mentir (chacune vient d'une erreur reelle de ce projet) :
  1. Un service ARRETE A LA DEMANDE n'est pas un incident. Le suivi video, la calibration, la navigation et le lidar ne demarrent plus au boot : leur absence est l'etat
     normal. Une tache STALE sur une part « a la demande » est ignoree, sauf si l'hote dit que le groupe tourne depuis longtemps (`demande_etat`).
  2. Une tache en defaut qui n'a AUCUNE part n'est jamais ignoree en silence : elle remonte dans `non_rattachees`. C'est ce qui dit que la nomenclature est incomplete.
  3. Un defaut doit durer `confirmations` relevees de suite avant de devenir un incident : un a-coup de charge n'en est pas un.
  4. Un incident resolu reste visible `garde_resolu_s` secondes, marque « resolu », pour que l'ecran ne clignote pas.
  5. La synthese que ce module produit pour /diagnostics est EXCLUE de l'analyse, sinon elle se verrait elle-meme en defaut (boucle).
"""
import time

OK, WARN, ERROR, STALE = 0, 1, 2, 3
NIVEAU = {OK: "OK", WARN: "WARN", ERROR: "ERROR", STALE: "STALE"}

# Gravite croissante d'un defaut, pour decider si une panne en amont en EXPLIQUE une en aval : WARN < STALE < ERROR.
RANG = {OK: 0, WARN: 1, STALE: 2, ERROR: 3}

# Tache publiee par incident_node lui-meme : jamais analysee.
SELF_TASK = "incident_node: synthese"

# Parts dont l'arret est l'etat normal (demarrees a la demande). Liste FERMEE : on n'en deduit pas de nouvelles.
A_LA_DEMANDE = ("bamboo_videotracking", "navigation", "laser", "cp2102_lidar")

# Fraicheur minimale (s) au-dela de laquelle « le groupe tourne » devient suspect si la tache reste muette.
RUNNING_TROP_LONGTEMPS_S = 60.0


def _leaf(name):
    """« /Robot/driver.esp32 (carte ESP32)/esp32_mavlink_driver/esp32_mavlink_driver: liaison serie » -> « esp32_mavlink_driver: liaison serie »."""
    return str(name).rsplit("/", 1)[-1]


def niveau_de(level):
    """Niveau en ENTIER. Dans rclpy le champ `level` de DiagnosticStatus est de type `byte` : il arrive comme `b'\\x02'` (bytes), pas comme `2`.
    Constate sur le vrai /diagnostics_agg le 2026-11-18 : `int(b'\\x00')` leve ValueError. Les tests construits avec des entiers ne pouvaient pas le voir."""
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else OK
    return int(level)


def feuilles(status_list):
    """Les taches FEUILLES de l'arbre agrege : celles dont le nom se termine par « noeud: tache ». Les noeuds intermediaires ne portent que le pire
    niveau de leurs enfants : on les ignore pour ne pas compter deux fois."""
    out = {}
    for s in status_list:
        nom = _leaf(getattr(s, "name", ""))
        if ": " not in nom:
            continue
        out[nom] = (niveau_de(getattr(s, "level", 0)), str(getattr(s, "message", "")), str(getattr(s, "hardware_id", "")))
    return out


def index_diag(doc):
    """tache de diagnostic -> [ids de parts qui la citent]."""
    idx = {}
    for p in doc["parts"]:
        for t in p.get("diag") or []:
            idx.setdefault(t, []).append(p["id"])
    return idx


def parts_en_defaut(feuilles_, idx, host_state=None, host_age=None):
    """-> (set de parts en defaut, {part: [taches]}, [taches en defaut sans part], [taches ignorees car attendues]).

    host_state / host_age : etat du groupe `tracking` cote hote (running / stopped...) et son anciennete ; sert a distinguer « arrete voulu » de « lance mais muet »."""
    bad, par_part, orphelines, ignorees = set(), {}, [], []
    for nom, (lvl, msg, _hw) in feuilles_.items():
        if nom == SELF_TASK or lvl == OK:
            continue
        ids = idx.get(nom)
        if not ids:
            orphelines.append(nom)
            continue
        for i in ids:
            attendu = i in A_LA_DEMANDE and lvl == STALE
            if attendu and i == "bamboo_videotracking" and host_state == "running" and host_age is not None and host_age > RUNNING_TROP_LONGTEMPS_S:
                attendu = False                                           # lance depuis plus de 60 s mais muet : ce n'est plus normal
            if attendu:
                ignorees.append(nom)
                continue
            bad.add(i)
            par_part.setdefault(i, []).append(nom)
    return bad, par_part, orphelines, sorted(set(ignorees))


def niveau_max(feuilles_, taches):
    """Pire niveau des taches citees (STALE compte pour 2.5 : plus grave qu'un WARN, a peu pres comme une ERROR)."""
    ordre = {OK: 0, WARN: 1, STALE: 2, ERROR: 3}
    pire = max((feuilles_[t][0] for t in taches if t in feuilles_), key=lambda l: ordre.get(l, 0), default=OK)
    return pire


class IncidentTracker:
    """Suit les incidents dans le temps : confirmation sur plusieurs relevees, memoire des resolus, anciennete."""

    def __init__(self, graph, confirmations=2, garde_resolu_s=30.0, clock=time.monotonic):
        self.g = graph
        self.confirmations = max(1, int(confirmations))
        self.garde_resolu_s = float(garde_resolu_s)
        self.clock = clock
        self._vu = {}            # part -> nombre de releves consecutifs en defaut
        self._actifs = {}        # racine -> dict incident (en cours)
        self._resolus = {}       # racine -> (incident, t_resolution)

    def update(self, bad, par_part, feuilles_):
        """bad : parts en defaut a ce releve. Rend (actifs, resolus) tries par criticite puis impact."""
        now = self.clock()
        for i in list(self._vu):
            if i not in bad:
                del self._vu[i]
        for i in bad:
            self._vu[i] = self._vu.get(i, 0) + 1
        confirmes = {i for i, n in self._vu.items() if n >= self.confirmations}
        rang = {i: RANG[niveau_max(feuilles_, par_part.get(i, []))] for i in confirmes}
        incidents = {}
        for racine, consequences, impact in self.g.incidents(confirmes, rang):
            part = self.g.parts[racine]
            taches = sorted({t for i in [racine] + consequences for t in par_part.get(i, [])})
            prev = self._actifs.get(racine)
            incidents[racine] = {
                "racine": racine, "label": part.get("label", racine), "type": part.get("type"), "criticite": part.get("criticite", 3),
                "niveau": NIVEAU[niveau_max(feuilles_, par_part.get(racine, []) or taches)],
                "depuis": prev["depuis"] if prev else now, "taches": taches,
                "consequences": consequences, "impact": impact, "certitude": part.get("certitude"),
                "consequences_detail": [{"id": c, "label": self.g.parts[c].get("label", c), "niveau": NIVEAU[niveau_max(feuilles_, par_part.get(c, []))],
                                         "taches": sorted(par_part.get(c, []))} for c in consequences],
                "resolution": list(part.get("resolution") or []), "ref": part.get("ref"),
            }
        for racine, inc in self._actifs.items():                       # disparus : passent en « resolu »
            if racine not in incidents:
                self._resolus[racine] = (inc, now)
        for racine in list(self._resolus):
            inc, t = self._resolus[racine]
            if racine in incidents or now - t > self.garde_resolu_s:    # revenu, ou trop ancien
                del self._resolus[racine]
        self._actifs = incidents
        actifs = sorted(incidents.values(), key=lambda x: (x["criticite"], -len(x["impact"])))
        resolus = [dict(inc, resolu_il_y_a_s=round(now - t, 1)) for inc, t in self._resolus.values()]
        for x in actifs:
            x["depuis_s"] = round(now - x["depuis"], 1)
        return actifs, resolus


def payload(actifs, resolus, orphelines, ignorees, n_taches):
    """Le JSON publie sur /bamboo/incidents. Les champs internes (`depuis`) sont retires."""
    def propre(x):
        return {k: v for k, v in x.items() if k != "depuis"}
    return {
        "version": 1,
        "n_incidents": len(actifs),
        "n_taches_analysees": n_taches,
        "incidents": [propre(x) for x in actifs],
        "resolus_recents": [propre(x) for x in resolus],
        "non_rattachees": sorted(orphelines),
        "ignorees_attendues": ignorees,
    }


def synthese(actifs, orphelines):
    """(niveau DiagnosticStatus, message court, [(cle, valeur)]) pour la tache standard `incident_node: synthese` de /diagnostics."""
    vals = [("incidents", str(len(actifs))), ("non_rattachees", str(len(orphelines)))]
    if not actifs and not orphelines:
        return OK, "aucun incident", vals
    if not actifs:
        return WARN, "%d tache(s) en defaut sans part dans la nomenclature : %s" % (len(orphelines), ", ".join(sorted(orphelines)[:3])), vals
    pire = actifs[0]
    niveau = ERROR if pire["criticite"] == 1 or pire["niveau"] in ("ERROR", "STALE") else WARN
    msg = "%d incident(s) : %s" % (len(actifs), "; ".join("%s%s" % (a["racine"], (" (" + ", ".join(a["consequences"][:3]) + ")") if a["consequences"] else "") for a in actifs[:3]))
    return niveau, msg, vals + [("racine_principale", pire["racine"])]
