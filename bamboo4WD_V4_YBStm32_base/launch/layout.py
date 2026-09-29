"""Chargeur du layout du robot : config/layout.yaml -> defauts des `enable_*` et du CABLAGE.

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

LA SECTION `wiring` (E5). Elle dit sous quel nom REEL circule chaque flux entre les modules.
Le nom LOGIQUE appartient au noeud (`servo_cmd`), le nom REEL au robot -- exactement le
partage de `servo_axes`, ou un producteur nomme un AXE et jamais une voie de carte. C'est ce
qui permet de brancher la meme carte sur une manette ici et sur un module de navigation
ailleurs sans toucher une ligne de code. Elle est OPTIONNELLE : absente, chaque module garde
le defaut declare dans son propre launch.
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


# Vocabulaire du cablage : groupe -> {nom LOGIQUE du flux: argument de launch du module}.
# Cette table EST la liste des flux recablables, et elle ne contient que ceux qui EXISTENT :
#   * `cmd_vel` n'y figure pas. Le driver l'expose bien (argument cmd_vel_topic), mais sa
#     souscription n'est creee qu'a l'armement moteur (_apply_actuation) -- donc hors scope
#     tant que T2/T3/T5 de bambooSTM32YB ne sont pas passes, et invisible a `node info`.
#   * le groupe `control` n'y figure pas non plus : teleop_node ne publie AUCUN cmd_vel.
# Y inscrire un flux inexistant donnerait un point de configuration qui a l'air de marcher.
WIRING_ARGS = {
    "driver": {"servo_cmd": "servo_cmd_topic"},
    "videotracking": {"servo_cmd": "servo_cmd_topic"},
}

# Repli quand la section manque : les noms LOGIQUES d'aujourd'hui, donc comportement
# inchange. Ces valeurs recopient les `default_value` des launch de module ; c'est une
# duplication assumee et bornee, et la ligne "cablage :" du journal affiche ce qui a
# reellement ete pousse, de sorte qu'une divergence se voit au lancement.
WIRING_FALLBACK = {
    "driver": {"servo_cmd": "/servo/cmd"},
    "videotracking": {"servo_cmd": "/servo/cmd"},
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


def loadWiring(path):
    """Lit la section `wiring` et renvoie ({groupe: {logique: reel}}, [lignes]).

    Section absente = cas normal, pas une erreur : on renvoie WIRING_FALLBACK. En revanche un
    groupe inconnu, un nom logique inconnu ou une valeur vide levent en NOMMANT le fautif. Un
    cablage avale en silence est le pire des cas : le bringup passe, le topic n'existe pas, et
    on cherche une panne du cote de DDS.
    """
    out = {group: dict(flows) for group, flows in WIRING_FALLBACK.items()}
    if not path or not os.path.isfile(path):
        return out, []

    with open(path, "r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle) or {}
    wiring = doc.get("wiring")
    if wiring is None:
        return out, ["cablage : section 'wiring' absente, noms logiques par defaut."]
    if not isinstance(wiring, dict):
        raise RuntimeError(
            "layout %s : section 'wiring' mal formee. Attendu une table groupe -> "
            "{nom logique: nom reel}. Groupes cablables : %s." % (
                path, ", ".join(sorted(WIRING_ARGS))))

    unknown = sorted(set(wiring) - set(WIRING_ARGS))
    if unknown:
        raise RuntimeError(
            "layout %s : wiring, groupe(s) non cablable(s) %s. Groupes cablables : %s. Un "
            "groupe sans flux recablable n'a rien a faire ici." % (
                path, ", ".join(unknown), ", ".join(sorted(WIRING_ARGS))))

    for group in sorted(wiring):
        flows = wiring[group] or {}
        if not isinstance(flows, dict):
            raise RuntimeError(
                "layout %s : wiring/%s mal forme. Attendu {nom logique: nom reel}." % (
                    path, group))
        bad = sorted(set(flows) - set(WIRING_ARGS[group]))
        if bad:
            raise RuntimeError(
                "layout %s : wiring/%s, flux inconnu(s) %s. Flux connus pour ce groupe : "
                "%s." % (path, group, ", ".join(bad),
                         ", ".join(sorted(WIRING_ARGS[group]))))
        for flow in sorted(flows):
            real = flows[flow]
            if not isinstance(real, str) or not real.strip():
                raise RuntimeError(
                    "layout %s : wiring/%s/%s vaut %r. Attendu un nom de topic non vide : "
                    "une chaine vide ferait retomber le module sur son defaut sans rien "
                    "dire." % (path, group, flow, real))
            out[group][flow] = real.strip()

    notes = []
    # Les deux bouts du chemin servo doivent porter le MEME nom : sinon la camera ne bouge
    # plus et aucun des deux noeuds ne se plaint -- le publieur publie, le driver ecoute
    # ailleurs. On le dit fort sans refuser : un jour, deux cartes pourraient se partager
    # les axes, et ce serait alors legitime.
    pub = out.get("videotracking", {}).get("servo_cmd")
    sub = out.get("driver", {}).get("servo_cmd")
    if pub != sub:
        notes.append(
            "ATTENTION : le tracking publie sur %s et la carte ecoute %s. Les servos ne "
            "bougeront pas, et aucun noeud ne s'en plaindra." % (pub, sub))
    return out, notes


def layoutDefaults(path):
    """({argument de launch: valeur}, [lignes a journaliser]) pour le bringup.

    Renvoie les `enable_*` ET les arguments de cablage, sous le nom que le bringup leur donne
    (`<groupe>_<flux>_topic`), parce que c'est lui qui les pousse aux modules.
    """
    groups, notes = loadLayout(path)
    flags = {GROUP_FLAGS[name]: ("true" if value else "false")
             for name, value in groups.items()}
    notes.append("composition : " + ", ".join(
        "%s=%s" % (name, "true" if groups[name] else "false") for name in sorted(groups)))

    wiring, wnotes = loadWiring(path)
    notes.extend(wnotes)
    for group in sorted(wiring):
        for flow in sorted(wiring[group]):
            flags["%s_%s_topic" % (group, flow)] = wiring[group][flow]
    notes.append("cablage : " + ", ".join(
        "%s/%s=%s" % (g, f, wiring[g][f])
        for g in sorted(wiring) for f in sorted(wiring[g])))
    return flags, notes
