"""Moteur de missions du robot : des CAPACITES enchainees (Python pur, sans ROS, horloge injectee).

Demande utilisateur du 2026-11-10 : le Command Center lance des MISSIONS, composees de CAPACITES
(relocalisation, exploration, navigation, acquisition, tracking, identification, calibrage...), elles-memes
disponibles par l'activation de services. Ex. « Exploration » = activer les services necessaires +
relocaliser + explorer.

Ce module ne sait RIEN de ROS : il recoit un modele (config/missions.yaml, deja charge en dict), l'etat du
robot, et rend des ACTIONS a executer (appeler un service...). Le noeud ROS (mission_node.py) execute ces
actions et lui renvoie les resultats. Resultat : tout ce qui est decision (ordre des etapes, prerequis,
delais, echec, annulation) est teste sans robot.

REGLES DE SECURITE (non negociables)
  * Une etape qui fait ROULER le robot (`arme: true`) ne part que si la traction est DEJA armee et la
    liaison du CC saine. Le moteur ne demande JAMAIS l'armement : « le robot ne s'arme jamais seul ».
  * Perte de liaison, desarmement ou arret demande : la mission est ANNULEE, et l'`annule` de l'etape en
    cours est execute.
  * Une etape qui depasse son `delai_s` fait ECHOUER la mission avec la raison ; rien ne reste « en cours ».
  * Une mission n'est lancee que si TOUS ses prerequis sont satisfaisables : sinon refus immediat, avec la
    cause, avant le moindre effet.
"""
import json
import re
import time

# Etats d'une mission
IDLE, RUNNING, DONE, FAILED, CANCELLED = "idle", "running", "done", "failed", "cancelled"
PAUSE_MAX_S = 600.0          # une etape en pause plus de 10 minutes est abandonnee (la mission est annulee)

# Etats d'une etape
PENDING, STARTING, WAITING_READY, RUNNING_STEP, STEP_DONE, PAUSED = (
    "pending", "starting", "waiting_ready", "running", "done", "paused")


class MissionError(ValueError):
    """Modele invalide ou mission impossible a composer."""


# ---------------------------------------------------------------------------------------------
# Modele : validation et composition
# ---------------------------------------------------------------------------------------------

def validate(model):
    """Verifie un modele ; leve MissionError avec TOUS les defauts trouves. Rend le modele."""
    errs = []
    caps = model.get("capacites") or {}
    missions = model.get("missions") or {}
    if not caps:
        errs.append("aucune capacite declaree")
    for name, c in caps.items():
        for dep in c.get("exige") or []:
            if dep not in caps:
                errs.append("capacite %s : prerequis inconnu %s" % (name, dep))
        for key in ("demarre", "lance", "annule"):
            v = c.get(key)
            if v is not None and (not isinstance(v, dict) or "service" not in v):
                errs.append("capacite %s : `%s` doit porter un `service`" % (name, key))
        if not c.get("demarre") and not c.get("lance") and not c.get("pret_quand") \
                and not c.get("fini_quand"):
            errs.append("capacite %s : rien a faire (ni demarre, ni lance, ni condition)" % name)
        if not isinstance(c.get("delai_s", 60), (int, float)) or c.get("delai_s", 60) <= 0:
            errs.append("capacite %s : delai_s doit etre positif" % name)
    for name, m in missions.items():
        steps = m.get("etapes") or []
        if not steps:
            errs.append("mission %s : aucune etape" % name)
        for s in steps:
            if s not in caps:
                errs.append("mission %s : capacite inconnue %s" % (name, s))
    # cycles de prerequis (les prerequis inconnus sont deja signales ci-dessus : on ne les suit pas)
    known = {n: {**c, "exige": [d for d in (c.get("exige") or []) if d in caps]}
             for n, c in caps.items()}
    for name in known:
        try:
            _closure(known, [name])
        except MissionError as e:
            errs.append(str(e))
    # services : tout service demande doit etre connu (socle ou optionnel), et les deux listes ne se recouvrent pas
    socle, opt = set(model.get("socle") or []), set(model.get("optionnels") or [])
    both = sorted(socle & opt)
    if both:
        errs.append("service a la fois socle et optionnel : %s" % ", ".join(both))
    for name, c in caps.items():
        for svc in c.get("services") or []:
            if svc not in socle and svc not in opt:
                errs.append("capacite %s : service %s absent du socle et des optionnels" % (name, svc))
    if errs:
        raise MissionError("; ".join(sorted(set(errs))))
    return model


def _closure(caps, wanted):
    """Capacites a derouler, prerequis d'abord, sans doublon. Leve MissionError sur un cycle."""
    order, state = [], {}

    def visit(n, path):
        if state.get(n) == 2:
            return
        if state.get(n) == 1:
            raise MissionError("cycle de prerequis : %s" % " -> ".join(path + [n]))
        state[n] = 1
        for dep in caps[n].get("exige") or []:
            visit(dep, path + [n])
        state[n] = 2
        order.append(n)
    for w in wanted:
        visit(w, [])
    return order


def compose(model, mission):
    """Liste ORDONNEE des capacites d'une mission, prerequis ajoutes devant, sans doublon."""
    caps = model["capacites"]
    m = model["missions"].get(mission)
    if m is None:
        raise MissionError("mission inconnue : %s" % mission)
    return _closure(caps, m["etapes"])


def services_needed(model, mission):
    """Services dont une mission a besoin : l'UNION de ceux de ses capacites (prerequis compris), sans doublon, dans l'ordre."""
    caps, out = model["capacites"], []
    for n in compose(model, mission):
        for svc in caps[n].get("services") or []:
            if svc not in out:
                out.append(svc)
    return out


def service_plan(model, mission, running):
    """Plan de services d'une mission, dans l'ORDRE d'execution.

    `running` : ensemble des services actuellement en marche. Rend {"stop": [...], "start": [...], "restore": [...]} :
      stop     services optionnels EN MARCHE que la mission n'a pas declares : a eteindre EN PREMIER (pas de surcharge CPU au demarrage)
      start    services declares par la mission qui ne tournent pas : a demarrer ENSUITE
      restore  ce qu'il faudra REMETTRE a la fin (= les services eteints par `stop`)
    Le socle n'est jamais dans `stop`, et un service hors socle et hors optionnels est ignore."""
    need = set(services_needed(model, mission))
    opt = [x for x in (model.get("optionnels") or [])]
    run = set(running or ())
    stop = [x for x in opt if x in run and x not in need]
    start = [x for x in services_needed(model, mission) if x not in run]
    return {"stop": stop, "start": start, "restore": list(stop)}


def required_params(model, mission):
    return list((model["missions"].get(mission) or {}).get("parametres") or [])


# ---------------------------------------------------------------------------------------------
# Conditions d'etat
# ---------------------------------------------------------------------------------------------

def check(cond, state):
    """Vrai si TOUTES les conditions de `cond` sont satisfaites dans `state` (dict a plat).

    Cles : `nav.loc_state` -> state["nav.loc_state"] doit etre dans la liste ; `noeud` -> le noeud doit
    etre dans state["noeuds"] ; `batt.pct_min` -> state["batt.pct"] >= seuil (inconnu = faux)."""
    if not cond:
        return True
    for key, want in cond.items():
        if key == "noeud":
            if want not in (state.get("noeuds") or ()):
                return False
        elif key == "batt.pct_min":
            v = state.get("batt.pct")
            if v is None or v < float(want):
                return False
        else:
            if state.get(key) not in want:
                return False
    return True


def why_not(cond, state):
    """Texte de ce qui manque pour que `cond` soit vraie (vide si elle l'est)."""
    out = []
    for key, want in (cond or {}).items():
        if key == "noeud":
            if want not in (state.get("noeuds") or ()):
                out.append("noeud %s absent" % want)
        elif key == "batt.pct_min":
            v = state.get("batt.pct")
            if v is None or v < float(want):
                out.append("batterie %s %% < %s %%" % ("?" if v is None else int(v), want))
        elif state.get(key) not in want:
            out.append("%s = %r (attendu %s)" % (key, state.get(key), "/".join(map(str, want))))
    return ", ".join(out)


def _subst(args, params):
    """Remplace `$nom` par la valeur du parametre de mission, dans les valeurs chaine d'un appel."""
    out = {}
    for k, v in (args or {}).items():
        if isinstance(v, str):
            v = re.sub(r"\$(\w+)", lambda m: str(params.get(m.group(1), "")), v)
        out[k] = v
    return out


# ---------------------------------------------------------------------------------------------
# Le moteur
# ---------------------------------------------------------------------------------------------

class MissionEngine:
    """Deroule UNE mission a la fois.

    Usage : `start()` puis `tick(now, state, link_ok, armed)` a ~2 Hz ; `tick` rend la liste des ACTIONS a
    executer : {"call": service, "args": {...}, "why": texte}. L'appelant les execute et peut appeler
    `call_result(ok, message)` pour les echecs d'appel. `cancel()` rend les actions d'annulation."""

    def __init__(self, model):
        self.model = validate(model)
        self.reset()

    def reset(self):
        self.status, self.name, self.params = IDLE, "", {}
        self.plan, self.i = [], 0
        self.step_state, self.step_t0, self.reason = PENDING, 0.0, ""
        self.started_at, self.ended_at, self.hold_t0, self.log = 0.0, 0.0, None, []
        self.svc_restore = []
        self.began = False
        self.pause_t0, self.pause_from = 0.0, ""

    # ---- consultation ----
    def current(self):
        return self.plan[self.i] if self.status == RUNNING and self.i < len(self.plan) else ""

    def snapshot(self, now=None):
        caps = self.model["capacites"]
        steps = []
        for k, n in enumerate(self.plan):
            st = ("done" if self.status == DONE or k < self.i else
                  self.step_state if (k == self.i and self.status == RUNNING) else
                  ("failed" if (k == self.i and self.status == FAILED) else
                   "cancelled" if (k == self.i and self.status == CANCELLED) else "pending"))
            steps.append({"id": n, "desc": caps[n].get("description", n),
                          "etat": st, "roule": bool(caps[n].get("arme"))})
        return {"etat": self.status, "mission": self.name, "pause": bool(self.status == RUNNING and self.step_state == PAUSED),
                "etapes": steps, "i": self.i,
                "raison": self.reason, "parametres": self.params,
                "duree_s": round(((self.ended_at or now) - self.started_at), 1) if (self.ended_at or now) and self.started_at else 0.0}

    def _note(self, now, text):
        self.log.append((round(now - self.started_at, 1) if self.started_at else 0.0, text))
        del self.log[:-40]

    # ---- lancement ----
    def preflight(self, mission, params, state, link_ok, armed):
        """Verifie TOUT avant d'agir. Rend (True, "") ou (False, raison). Aucun effet."""
        if self.status == RUNNING:
            return False, "une mission est deja en cours : %s" % self.name
        try:
            plan = compose(self.model, mission)
        except MissionError as e:
            return False, str(e)
        missing = [p for p in required_params(self.model, mission) if not (params or {}).get(p)]
        if missing:
            return False, "parametre manquant : %s" % ", ".join(missing)
        caps = self.model["capacites"]
        roule = [n for n in plan if caps[n].get("arme")]
        if roule:
            if not link_ok:
                return False, "liaison du CC absente (page ou video) : le robot roulerait sans surveillance"
            if not armed:
                return False, "traction non armee : armer d'abord (le robot ne s'arme jamais seul)"
        return True, ""

    def start(self, mission, params, now, state, link_ok, armed):
        ok, why = self.preflight(mission, params, state, link_ok, armed)
        if not ok:
            return False, why, []
        self.reset()
        self.status, self.name, self.params = RUNNING, mission, dict(params or {})
        self.plan = compose(self.model, mission)
        self.i, self.started_at = 0, now
        self.step_state, self.step_t0 = PENDING, now
        self._note(now, "mission %s : %s" % (mission, " > ".join(self.plan)))
        # SERVICES : on eteint d'abord le superflu, on demarre ensuite le necessaire. Les services en marche viennent de `state`.
        plan = service_plan(self.model, mission, state.get("services"))
        self.svc_restore = plan["restore"]
        acts = []
        if plan["stop"]:
            acts.append({"services": "stop", "names": plan["stop"], "why": "eteindre les services inutiles : %s" % ", ".join(plan["stop"])})
            self._note(now, "services eteints : %s" % ", ".join(plan["stop"]))
        if plan["start"]:
            acts.append({"services": "start", "names": plan["start"], "why": "demarrer les services necessaires : %s" % ", ".join(plan["start"])})
        return True, "", acts

    def restore_actions(self):
        """Services eteints au lancement, a REMETTRE (une seule fois) quand la mission se termine, quelle qu'en soit l'issue."""
        names, self.svc_restore = list(getattr(self, "svc_restore", []) or []), []
        return [{"services": "start", "names": names, "why": "remettre les services eteints au lancement : %s" % ", ".join(names)}] if names else []

    # ---- cancel ----
    def cancel(self, now, why="arret demande"):
        """Annule la mission en cours. Rend les actions d'annulation (etape en cours et deja lancees)."""
        if self.status != RUNNING:
            return []
        acts = self._abort_actions(upto=self.i)
        self.status, self.reason, self.ended_at = CANCELLED, why, now
        self._note(now, "ANNULEE : %s" % why)
        return acts + self.restore_actions()

    def _abort_actions(self, upto):
        """Appels `annule` des etapes lancees (de la plus recente a la plus ancienne), sans doublon."""
        caps, seen, out = self.model["capacites"], set(), []
        for k in range(min(upto, len(self.plan) - 1), -1, -1):
            c = caps[self.plan[k]]
            a = c.get("annule")
            # on n'annule que les actions qui ROULENT ou qui ont ete lancees ; pas la pile NAV, qui reste
            # allumee (regle utilisateur : NAV ne s'eteint que sur un arret explicite).
            if not a or (k != upto and not c.get("arme")) or (c.get("demarre") and not c.get("arme")):
                continue
            key = (a["service"], json.dumps(a.get("args") or {}, sort_keys=True))
            if key in seen:
                continue
            seen.add(key)
            out.append({"call": a["service"], "args": _subst(a.get("args"), self.params),
                        "why": "annulation de %s" % self.plan[k]})
        return out

    def _fail(self, now, why):
        acts = self._abort_actions(upto=self.i)
        self.status, self.reason, self.ended_at = FAILED, why, now
        self._note(now, "ECHEC : %s" % why)
        return acts + self.restore_actions()

    # ---- avancement ----
    def tick(self, now, state, link_ok, armed):
        """Un pas. Rend la liste d'actions a executer (jamais None)."""
        if self.status != RUNNING:
            return []
        caps = self.model["capacites"]
        n = self.plan[self.i]
        c = caps[n]
        # PAUSE (2026-11-11) : une etape qui ROULE et dont la traction tombe n'est PAS annulee, elle est mise en pause. La traction peut
        # tomber par la perte de la manette ou du CC (armement simple), Echap ou B (arret d'urgence) : le robot est deja arrete par
        # le teleop, et le but de navigation annule. Au reappui (traction de nouveau armee, simple ou verrouillee) l'etape REPREND
        # ou elle en etait : on relance son action depuis la position actuelle. Seul le bouton « Annuler » abandonne la mission.
        # Aucun test supplementaire a la reprise (decision de l'utilisateur) : la traction armee est l'autorisation.
        if self.step_state == PAUSED:
            if armed:
                self._note(now, "reprise de %s (traction armee)" % n)
                self.step_state, self.step_t0, self.hold_t0 = PENDING, now, None
                self.pause_t0 = 0.0
                return self.tick(now, state, link_ok, armed)          # relance l'etape depuis le debut de son action
            if now - self.pause_t0 > PAUSE_MAX_S:
                return self._cancel_with(now, "%s : pause de plus de %d min sans reprise" % (n, PAUSE_MAX_S // 60))
            return []
        if c.get("arme") and self.step_state in (RUNNING_STEP, STARTING) and not armed:
            self.pause_t0, self.pause_from = now, self.step_state
            self.step_state = PAUSED
            self._note(now, "PAUSE de %s : traction desarmee" % n)
            return []
        if c.get("arme") and self.step_state in (RUNNING_STEP, STARTING):
            watch = c.get("surveille")
            if watch and not check(watch, state):
                return self._cancel_with(now, "%s : %s" % (n, why_not(watch, state)))
        acts = []
        if self.step_state == PENDING:
            if c.get("arme") and not armed:
                # Pas encore de traction : l'etape ATTEND au lieu d'annuler la mission (relance des que la traction est armee).
                self.pause_t0, self.pause_from = now, PENDING
                self.step_state = PAUSED
                self._note(now, "PAUSE de %s : traction non armee" % n)
                return []
            guard = c.get("garde")
            if guard and not check(guard, state):
                return self._cancel_with(now, "%s refusee : %s" % (n, why_not(guard, state)))
            self.step_t0, self.hold_t0 = now, None
            if c.get("demarre"):
                d = c["demarre"]
                acts.append({"call": d["service"], "args": _subst(d.get("args"), self.params),
                             "why": "demarrer %s" % n})
                self.step_state = WAITING_READY
            else:
                self.step_state = WAITING_READY
            self._note(now, "etape %s" % n)
            return acts
        # delai
        limit = float(c.get("delai_s", 60))
        if now - self.step_t0 > limit:
            what = why_not(c.get("pret_quand") if self.step_state == WAITING_READY
                           else c.get("fini_quand"), state)
            return self._fail_acts(now, "%s : delai de %d s depasse%s"
                                   % (n, limit, " (%s)" % what if what else ""))
        if self.step_state == WAITING_READY:
            if not check(c.get("pret_quand"), state):
                return []
            # prete : on lance l'action
            if c.get("lance"):
                l = c["lance"]
                acts.append({"call": l["service"], "args": _subst(l.get("args"), self.params),
                             "why": "lancer %s" % n})
                self.step_state = RUNNING_STEP
                self.began = False
                # le delai se compte maintenant depuis le LANCEMENT : l'attente de la pile n'en mange pas
                self.step_t0 = now
                return acts
            self.step_state = RUNNING_STEP
        if self.step_state == RUNNING_STEP:
            # DEBUT (2026-11-11) : une action qui ROULE doit d'abord être VUE en marche (`debut_quand`). Sans cela, un arrêt
            # immédiat du superviseur (ex. « sortie du mode NAV ») ressemble à une fin (`auto_state off`) : mission « terminée » sans avoir roulé.
            beg = c.get("debut_quand")
            if beg and not self.began:
                if check(beg, state):
                    self.began = True
                    self._note(now, "%s : action demarree" % n)
                elif now - self.step_t0 > float(c.get("debut_delai_s", 10)):
                    why = str(state.get("nav.detail") or "").strip()
                    return self._fail_acts(now, "%s : l'action n'a pas demarre%s" % (n, " (%s)" % why if why else ""))
                else:
                    return acts
            done = check(c.get("fini_quand"), state) if c.get("fini_quand") else True
            if done and self._settled(c, now):
                hold = float(c.get("maintien_s", 0) or 0)
                if hold > 0:
                    # La condition doit TENIR : une localisation « ok » qui retombe avant `hold` n'est pas une fin.
                    if self.hold_t0 is None:
                        self.hold_t0 = now
                    if now - self.hold_t0 < hold:
                        return acts
                return self._next(now)
            self.hold_t0 = None
        return acts

    def _settled(self, c, now):
        """Une condition d'etat deja vraie AVANT la fin du lancement n'est pas une fin : il faut qu'un
        tick au moins se soit ecoule depuis le lancement (la reponse du superviseur arrive apres)."""
        return now - self.step_t0 >= 1.0 or not c.get("lance")

    def _next(self, now):
        self._note(now, "etape %s terminee" % self.plan[self.i])
        self.i += 1
        if self.i >= len(self.plan):
            self.status, self.ended_at = DONE, now
            self._note(now, "mission terminee")
            return self.restore_actions()
        self.step_state, self.step_t0, self.hold_t0 = PENDING, now, None
        return []

    def _cancel_with(self, now, why):
        return self.cancel(now, why)

    def _fail_acts(self, now, why):
        return self._fail(now, why)

    def call_result(self, now, ok, message):
        """Un appel de service a ECHOUE (refus du superviseur, service absent) : la mission echoue."""
        if self.status != RUNNING or ok:
            return []
        return self._fail(now, "%s : %s" % (self.plan[self.i], message or "appel refuse"))
