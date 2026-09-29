"""Chargeur de la COMPOSITION du robot : config/layout.yaml -> defauts des `enable_*`.

POURQUOI CE FICHIER EXISTE. Avant E3, la composition de la machine etait ecrite en Python,
dans les `default_value` du bringup : retirer le groupe video demandait d'editer un launch.
Ce n'est pas une donnee de code, c'est une donnee de ROBOT -- au meme titre que sa geometrie.
Elle vit donc dans config/, dans le paquet DE CE ROBOT, et le Python n'en garde que la
mecanique de lecture.

CE QU'IL NE FAIT PAS. Il ne cherche aucun fichier : le bringup lui POUSSE le chemin, comme
il pousse le canonique aux modules (invariant pose a E2). Il ne parle a aucune carte, ne
lance aucun noeud, et ne connait des groupes que leurs NOMS -- le paquet de chacun reste
nomme dans le bringup.

TROIS FACONS D'ECHOUER, TOUTES BRUYANTES. Un groupe inconnu, une cle `enable` absente, une
valeur qui n'est pas un booleen YAML : chacune leve en NOMMANT le fichier et le groupe. Le
cas dangereux n'est pas l'erreur, c'est le silence -- un groupe ignore laisserait le bringup
passer, et on chercherait la panne du cote de DDS.

Fichier ABSENT, en revanche, n'est pas une erreur : c'est le cas d'un paquet fraichement
construit. On retombe alors sur FALLBACK, qui reproduit le comportement d'avant E3.
"""
import os

# python3-yaml est une dependance dure de ros2launch : toujours presente la ou ce module
# tourne (meme raisonnement que bamboo_base/capability_check.py).
import yaml

# Vocabulaire des groupes : nom dans layout.yaml -> argument de launch du bringup. Cette
# table EST la liste des groupes connus ; elle n'est pas deductible mecaniquement, le groupe
# `videotracking` (nom du paquet et du launch) portant l'argument `enable_tracking`.
GROUP_FLAGS = {
    "driver": "enable_driver",
    "description": "enable_description",
    "video": "enable_video",
    "videotracking": "enable_tracking",
    "control": "enable_control",
}

# Repli quand le fichier manque. Identique aux defauts d'avant E3, donc TOUT GROUPE QUI
# COMMANDE UN ACTIONNEUR EST A false : `videotracking` et `control` bougent les servos, et
# un layout absent ne doit jamais etre une facon detournee de les armer.
FALLBACK = {
    "driver": True,
    "description": True,
    "video": True,
    "videotracking": False,
    "control": False,
}


def loadLayout(path):
    """Lit `path` et renvoie ({groupe: bool}, [lignes a journaliser])."""
    if not path or not os.path.isfile(path):
        return dict(FALLBACK), [
            "layout absent (%s) : defauts de repli, tout groupe d'actionneur a false." % path]

    with open(path, "r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle) or {}
    if not isinstance(doc, dict) or not isinstance(doc.get("groups"), dict):
        raise RuntimeError(
            "layout %s : section 'groups' absente ou mal formee. Attendu une table "
            "groupe -> {enable: true|false}. Groupes connus : %s." % (
                path, ", ".join(sorted(GROUP_FLAGS))))
    groups = doc["groups"]

    unknown = sorted(set(groups) - set(GROUP_FLAGS))
    if unknown:
        raise RuntimeError(
            "layout %s : groupe(s) inconnu(s) %s. Groupes connus : %s. Refuse a dessein : "
            "un groupe ignore en silence laisserait le bringup passer avec un noeud en "
            "moins." % (path, ", ".join(unknown), ", ".join(sorted(GROUP_FLAGS))))

    out = dict(FALLBACK)
    for name in sorted(groups):
        spec = groups[name]
        if not isinstance(spec, dict) or "enable" not in spec:
            raise RuntimeError(
                "layout %s : groupe '%s' sans cle 'enable'. Ecrire {enable: true} ou "
                "{enable: false}." % (path, name))
        value = spec["enable"]
        if not isinstance(value, bool):
            raise RuntimeError(
                "layout %s : 'enable' du groupe '%s' vaut %r. Attendu un booleen YAML "
                "(true / false) et non une chaine : \"false\" serait vrai." % (
                    path, name, value))
        out[name] = value

    notes = []
    absent = sorted(set(GROUP_FLAGS) - set(groups))
    if absent:
        notes.append("groupe(s) non nomme(s) dans le layout, donc au defaut de repli : %s."
                     % ", ".join(absent))
    return out, notes


def layoutDefaults(path):
    """({argument de launch: "true"|"false"}, [lignes a journaliser]) pour le bringup."""
    groups, notes = loadLayout(path)
    flags = {GROUP_FLAGS[name]: ("true" if value else "false")
             for name, value in groups.items()}
    notes.append("composition : " + ", ".join(
        "%s=%s" % (name, "true" if groups[name] else "false") for name in sorted(groups)))
    return flags, notes
