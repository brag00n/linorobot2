"""Carte des touches de la manette PAR MODE et PAR GROUPE (joy_bamboo.yaml, `keymap.*`).

Python PUR, teste sans ROS. Forme des parametres (aplatis par ROS) :

    keymap.<mode>.groups              "vision;cible"      groupes, ordre de la croix haut/bas
    keymap.<mode>.<groupe>.<TOUCHE>   "acquisition"       appui court (ou unique)
    keymap.<mode>.<groupe>.<TOUCHE>_long  "recenter"      appui long sur la meme touche

TOUCHES CONFIGURABLES : A X Y L R ZL ZR MINUS PLUS. B et la croix sont GLOBAUX (STOP,
armement, changement de mode et de groupe) et NON reaffectables : c'est la securite, et
une carte qui les reprendrait est refusee au demarrage.

Une touche qui a un appui long part au RELACHEMENT pour son appui court (on ne peut pas
savoir avant qu'un appui sera court) ; une touche SANS appui long part au front, comme le
prototype. D'ou la regle : ne mettre en appui court d'une touche a double usage qu'une
action qui supporte d'attendre le relachement.
"""

KEYS = ("A", "X", "Y", "L", "R", "ZL", "ZR", "MINUS", "PLUS")
GLOBAL_KEYS = ("B", "DPAD")
LONG = "_long"
PREFIX = "keymap"


def _items(text):
    return [t.strip() for t in str(text or "").split(";") if t.strip()]


def parseKeymap(params, actions, modes=None):
    """{nom aplati: valeur} -> {mode: {"groups": [...], "bind": {groupe: {cle: action}}}}.

    `actions` = noms d'action connus du noeud. `modes` (facultatif) = modes connus : une
    carte pour un mode inconnu est une faute de frappe, refusee. Leve ValueError en nommant
    la faute.
    """
    out = {}
    for name, value in params.items():
        parts = name.split(".")
        if parts[0] != PREFIX:
            continue
        if len(parts) == 3 and parts[2] == "groups":
            mode = parts[1]
            groups = _items(value)
            if not groups:
                raise ValueError("%s : aucun groupe." % name)
            if len(set(groups)) != len(groups):
                raise ValueError("%s : groupe en double." % name)
            out.setdefault(mode, {"groups": [], "bind": {}})["groups"] = groups
    for name, value in params.items():
        parts = name.split(".")
        if parts[0] != PREFIX or (len(parts) == 3 and parts[2] == "groups"):
            continue
        if len(parts) != 4:
            raise ValueError("%s : attendu keymap.<mode>.<groupe>.<TOUCHE>." % name)
        _, mode, group, key = parts
        base = key[:-len(LONG)] if key.endswith(LONG) else key
        if base in GLOBAL_KEYS:
            raise ValueError("%s : %s est une touche GLOBALE (STOP/armement/croix), non "
                             "reaffectable." % (name, base))
        if base not in KEYS:
            raise ValueError("%s : touche inconnue %s (connues : %s)."
                             % (name, base, " ".join(KEYS)))
        if mode not in out:
            raise ValueError("%s : keymap.%s.groups absent." % (name, mode))
        if group not in out[mode]["groups"]:
            raise ValueError("%s : groupe %s absent de keymap.%s.groups." % (name, group, mode))
        action = str(value).strip()
        if action not in actions:
            raise ValueError("%s : action inconnue '%s' (connues : %s)."
                             % (name, action, ", ".join(sorted(actions))))
        out[mode]["bind"].setdefault(group, {})[key] = action
    if modes is not None:
        unknown = [m for m in out if m not in modes]
        if unknown:
            raise ValueError("keymap pour des modes inconnus : %s (connus : %s)."
                             % (", ".join(unknown), ", ".join(modes)))
    return out


def displayKey(key):
    """'X_long' -> 'X long', 'MINUS' -> '-' : le texte du HUD."""
    long_ = key.endswith(LONG)
    base = key[:-len(LONG)] if long_ else key
    base = {"MINUS": "-", "PLUS": "+"}.get(base, base)
    return base + (" long" if long_ else "")
