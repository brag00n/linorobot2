"""parts_model.py : modele de la nomenclature des parts du robot (parts.yaml). Python PUR, sans ROS : teste hors du robot.

Deux relations, qu'il ne faut pas confondre :
  parent      hierarchie d'appartenance (un seul parent, arbre) ;
  depends_on  dependance de fonctionnement (graphe oriente, doit rester SANS cycle).

Ce module donne les trois operations dont l'analyse d'incident a besoin :
  causes(id)   les parts en AMONT dont `id` depend, transitivement (d'ou peut venir sa panne) ;
  impact(id)   les parts en AVAL qui dependent de `id`, transitivement (ce qui tombe avec lui) ;
  racines(ids) parmi des parts EN DEFAUT, celles dont aucune cause amont n'est elle-meme en defaut : les vraies racines de l'incident.
"""
import re
from collections import defaultdict

TYPES = ("physique", "electronique", "logiciel", "infrastructure", "systeme")
CERTITUDES = ("confirme", "deduit", "a_renseigner")
ID_RE = re.compile(r"^[a-z0-9_]+$")


class PartsError(ValueError):
    pass


def load(path):
    import yaml
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def validate(doc, urdf_names=None, diag_names=None):
    """Rend la liste de TOUS les defauts (jamais seulement le premier). Vide = nomenclature saine.

    urdf_names : ensemble des noms de liens et d'articulations de l'URDF ; si fourni, tout id physique qui porte le nom d'un lien doit exister.
    diag_names : ensemble des taches de diagnostic reellement publiees ; si fourni, toute reference `diag:` doit exister."""
    errs = []
    parts = doc.get("parts") or []
    ids = [p.get("id") for p in parts]
    seen = set()
    for i in ids:
        if i in seen:
            errs.append("id en double : %s" % i)
        seen.add(i)
    idset = set(ids)
    racines = [p["id"] for p in parts if not p.get("parent")]
    if len(racines) != 1:
        errs.append("il faut exactement UNE racine (part sans parent), trouve %d : %s" % (len(racines), racines))
    for p in parts:
        i = p.get("id")
        if not i or not ID_RE.match(str(i)):
            errs.append("id invalide : %r (minuscules, chiffres, _)" % i)
            continue
        if p.get("type") not in TYPES:
            errs.append("%s : type %r inconnu (attendu : %s)" % (i, p.get("type"), ", ".join(TYPES)))
        if p.get("certitude") not in CERTITUDES:
            errs.append("%s : certitude %r inconnue (attendu : %s)" % (i, p.get("certitude"), ", ".join(CERTITUDES)))
        if p.get("criticite") not in (1, 2, 3):
            errs.append("%s : criticite %r invalide (1, 2 ou 3)" % (i, p.get("criticite")))
        if not p.get("label"):
            errs.append("%s : label manquant" % i)
        par = p.get("parent")
        if par and par not in idset:
            errs.append("%s : parent inconnu %r" % (i, par))
        if par == i:
            errs.append("%s : parent de lui-meme" % i)
        for d in p.get("depends_on") or []:
            if d not in idset:
                errs.append("%s : depends_on inconnu %r" % (i, d))
            if d == i:
                errs.append("%s : depend de lui-meme" % i)
        # une part « confirme » doit dire d'ou vient l'information ; une reference vide ne peut pas etre « confirme »
        if p.get("certitude") == "confirme" and not p.get("source"):
            errs.append("%s : certitude « confirme » sans source" % i)
        if p.get("certitude") == "a_renseigner" and p.get("ref"):
            errs.append("%s : « a_renseigner » mais une reference est donnee : corriger la certitude" % i)
        if p.get("type") == "physique" and p.get("ref") is None and p.get("certitude") == "confirme":
            errs.append("%s : physique confirme sans reference" % i)
        if urdf_names is not None and p.get("type") == "physique" and re.search(r"(_link|_joint)$|^laser$", str(i)) and i not in urdf_names:
            errs.append("%s : nom de lien/joint absent de l'URDF" % i)
        if diag_names is not None:
            for t in p.get("diag") or []:
                if t not in diag_names:
                    errs.append("%s : tache de diagnostic inconnue %r" % (i, t))
    errs += _cycles(parts, "parent")
    errs += _cycles(parts, "depends_on")
    return errs


def _cycles(parts, key):
    """Detecte un cycle dans la relation `key` (parent ou depends_on)."""
    g = {p["id"]: ([p[key]] if key == "parent" and p.get(key) else list(p.get(key) or [])) for p in parts if p.get("id")}
    state, errs = {}, []

    def visit(n, path):
        state[n] = 1
        for m in g.get(n, ()):
            if m not in g:
                continue
            if state.get(m) == 1:
                errs.append("cycle de %s : %s" % (key, " -> ".join(path + [n, m])))
            elif state.get(m) is None:
                visit(m, path + [n])
        state[n] = 2

    for n in g:
        if state.get(n) is None:
            visit(n, [])
    return errs


class Graph:
    def __init__(self, doc):
        self.parts = {p["id"]: p for p in doc["parts"]}
        self.deps = {i: list(p.get("depends_on") or []) for i, p in self.parts.items()}
        self.aval = defaultdict(list)
        for i, ds in self.deps.items():
            for d in ds:
                self.aval[d].append(i)

    def causes(self, i):
        """Parts en amont de `i` (transitif), sans `i`."""
        out, pile = [], list(self.deps.get(i, ()))
        while pile:
            n = pile.pop()
            if n not in out:
                out.append(n)
                pile += self.deps.get(n, ())
        return out

    def impact(self, i):
        """Parts en aval de `i` (transitif) : ce qui ne fonctionne plus si `i` tombe."""
        out, pile = [], list(self.aval.get(i, ()))
        while pile:
            n = pile.pop()
            if n not in out:
                out.append(n)
                pile += self.aval.get(n, ())
        return out

    def racines(self, en_defaut):
        """Parmi les parts EN DEFAUT, celles dont aucune cause amont n'est elle-meme en defaut."""
        bad = set(en_defaut)
        return sorted(i for i in bad if not (set(self.causes(i)) & bad))

    def incidents(self, en_defaut):
        """[(racine, [consequences en defaut], [impact total])], le plus critique d'abord (criticite 1 avant 3, puis plus d'impact)."""
        bad = set(en_defaut)
        out = []
        for r in self.racines(bad):
            cons = [x for x in self.impact(r) if x in bad]
            out.append((r, sorted(cons), sorted(self.impact(r))))
        out.sort(key=lambda t: (self.parts[t[0]].get("criticite", 3), -len(t[2])))
        return out
