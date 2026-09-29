"""Ou vit le fichier canonique d'un robot -- une seule reponse, pour tous les modules.

POURQUOI CE FICHIER EXISTE (etape E2, 2026-09-28)
-------------------------------------------------
La configuration d'un robot appartient au paquet DE CE ROBOT : celle de bambooSTM32YB a
quitte bamboo_base pour bamboo4WD_V4_YBStm32_base/config/. Les robots qui n'ont pas encore
leur propre paquet (WSEsp32) gardent la leur dans config/robots/ d'ici. Il y a donc DEUX
emplacements possibles, et sans un point unique pour les connaitre, quatre launch de module
auraient recopie la meme liste -- donc auraient divergé au premier deplacement suivant.

L'ORDRE DE PRIORITE, ET CE QU'IL GARANTIT
-----------------------------------------
  1. le chemin POUSSE par le bringup du robot (argument `robot_config`) : il fait autorite,
     sans aucune recherche. C'est le chemin normal, et l'invariant du plan : un module ne va
     jamais chercher la configuration d'un robot, on la lui donne.
  2. <robot>_base/config/<robot>.yaml, par CONVENTION de nom -- pas par une liste de robots,
     que ce paquet generique n'a pas a porter.
  3. bamboo_base/config/robots/<robot>.yaml, l'ancien emplacement.

La recherche (2 et 3) n'est PAS le chemin normal : elle sert aux services Docker qui lancent
un module DIRECTEMENT, sans bringup (`bamboo_video`, `bamboo_videotracking`, `driver.stm32`),
et aux essais en ligne de commande.

Chaque emplacement est essaye DEUX fois, par l'index ament puis dans l'arbre SOURCE : ces
conteneurs ne construisent qu'une poignee de paquets (`--packages-select`), donc l'index
ament ignore le paquet du robot -- alors que l'arbre source, lui, est bind-monte en entier.

Aucun repli SILENCIEUX : si rien n'existe, on leve en NOMMANT tous les chemins essayes. Un
robot qui demarre sur des valeurs physiques par defaut roule faux sans se plaindre, ce qui
est precisement le defaut corrige a E1.
"""
import os

# <install|src>/bamboo_base/bamboo_base/ -> racine du depot dans l'arbre source
_HERE = os.path.dirname(os.path.realpath(__file__))
_SRC_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))


def robotConfigCandidates(robot):
    """Liste ordonnee des chemins ou le fichier canonique de `robot` peut vivre."""
    homes = ((robot + "_base", os.path.join("config", robot + ".yaml")),
             ("bamboo_base", os.path.join("config", "robots", robot + ".yaml")))
    out = []
    for package, rel in homes:
        try:
            from ament_index_python.packages import get_package_share_directory
            out.append(os.path.join(get_package_share_directory(package), rel))
        except Exception:
            # Paquet inconnu de l'index : normal dans un conteneur qui ne le construit pas.
            pass
        out.append(os.path.join(_SRC_ROOT, package, rel))
    return out


def resolveRobotConfig(robot, pushed=""):
    """Renvoie le chemin du fichier canonique. Leve en nommant les candidats si absent."""
    if pushed:
        # Pousse : on ne cherche pas, et on ne se rabat pas en silence sur un autre fichier.
        # Un chemin pousse faux doit se voir tout de suite, pas se faire remplacer.
        if not os.path.isfile(pushed):
            raise RuntimeError(
                "fichier canonique pousse introuvable : %s -- verifier l'argument "
                "robot_config du bringup." % pushed)
        return pushed

    candidates = robotConfigCandidates(robot)
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise RuntimeError(
        "fichier canonique du robot '%s' introuvable. Cherche, dans l'ordre :\n  %s\n"
        "La configuration d'un robot vit dans <robot>_base/config/ ; passer robot_config:="
        "<chemin> pour la nommer explicitement." % (robot, "\n  ".join(candidates)))
