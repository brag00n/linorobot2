"""Modele des modes du robot (config/modes.yaml) : lecture et validation, en Python PUR.

Separe de mode_manager_node pour etre teste sans ROS : un modele faux doit etre refuse au
DEMARRAGE du gestionnaire, pas decouvert au premier appui sur la croix.
"""

FIELDS = ("label", "requires", "enter_modecmd", "exit_modecmd",
          "enter_params", "exit_params", "exit_cancel", "ondemand")
NEXT, PREV = "__next__", "__previous__"


def _items(text):
    return [t.strip() for t in str(text or "").split(";") if t.strip()]


def parseModel(names, raw):
    """{mode: {champ: valeur brute}} -> modele valide, ou ValueError nommant la faute.

    Fonction PURE (testee sans ROS) : un modele faux doit etre refuse au DEMARRAGE, pas
    decouvert au premier appui sur la croix.
    """
    if not names:
        raise ValueError("`modes` est vide : au moins un mode est requis.")
    if len(set(names)) != len(names):
        raise ValueError("`modes` contient un doublon : %s" % names)
    model = {}
    for name in names:
        if name.startswith("__"):
            raise ValueError("nom de mode reserve : %s" % name)
        r = raw.get(name, {})
        m = {"label": str(r.get("label") or name.upper())}
        m["requires"] = _items(r.get("requires"))
        for req in m["requires"]:
            if not (req.startswith("node:/") or req.startswith("param:")):
                raise ValueError("%s.requires : '%s' n'est ni `node:/nom` ni `param:nom`."
                                 % (name, req))
        for f in ("enter_modecmd", "exit_modecmd"):
            m[f] = []
            for it in _items(r.get(f)):
                if "=" not in it:
                    raise ValueError("%s.%s : '%s' attend `cible=valeur`." % (name, f, it))
                tgt, val = it.split("=", 1)
                if not val:
                    # Jamais de valeur vide : les consommateurs ne s'accordent pas dessus
                    # (cf. l'en-tete de teleop_node.py).
                    raise ValueError("%s.%s : valeur vide pour '%s'." % (name, f, tgt))
                m[f].append((tgt.strip(), val.strip()))
        for f in ("enter_params", "exit_params"):
            m[f] = []
            for it in _items(r.get(f)):
                if not it.startswith("/") or ":" not in it or "=" not in it:
                    raise ValueError("%s.%s : '%s' attend `/noeud:param=valeur`."
                                     % (name, f, it))
                node, rest = it.split(":", 1)
                prm, val = rest.split("=", 1)
                m[f].append((node.strip(), prm.strip(), val.strip()))
        m["exit_cancel"] = _items(r.get("exit_cancel"))
        # Groupe de services A LA DEMANDE (docker/ondemand/groups.conf) : entrer dans le mode
        # le demarre, en sortir l'arrete apres un delai. Ses `node:` absents ne rendent alors
        # plus le mode indisponible -- ils sont justement ce qu'on va demarrer.
        m["ondemand"] = str(r.get("ondemand") or "").strip()
        if m["ondemand"] and not all(c.isalnum() or c == "_" for c in m["ondemand"]):
            raise ValueError("%s.ondemand : '%s' n'est pas un nom de groupe (lettres, chiffres,"
                             " _)." % (name, m["ondemand"]))
        model[name] = m
    return model
