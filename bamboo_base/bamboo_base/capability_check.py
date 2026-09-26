#!/usr/bin/env python3
"""Confronte la table `servo_axes` d'un robot aux CAPACITES declarees par ses controleurs.

POURQUOI. La table `servo_axes` du fichier canonique est le seul endroit qui lie un axe
(`pan`, `tilt`) a une carte et a une voie. Rien, jusqu'ici, ne verifiait que la carte
designee sait reellement commander un servo PWM -- et le parc contient justement une carte
qui n'en a AUCUN : la WaveShare ESP32 repond MAV_RESULT_UNSUPPORTED a DO_SET_SERVO
(ConnectorMavlink.cpp:450) en renvoyant malgre tout un ACK. Une consigne y partirait donc
dans le vide, sans erreur, sans mouvement, et le diagnostic serait cherche du cote de la
mecanique. C'est le piege du chantier 1, paye une fois.

Ce module le rend IMPOSSIBLE A MANQUER : il echoue en NOMMANT l'axe, le controleur et la
capacite manquante, au bringup, avant qu'aucun servo ne soit commande.

CE QU'IL NE FAIT PAS : parler a une carte. Il ne lit que des fichiers -- le canonique du
robot et le capabilities.yaml de chaque module de controleur. Il peut donc tourner pendant
que le driver tient le port serie, qui est MONO-PROPRIETAIRE.

Une capacite declaree n'est pas une capacite mesuree : ce fichier attrape les erreurs de
CABLAGE LOGIQUE (un axe sur la mauvaise carte), pas un servo debranche.

Usage :
    ros2 run bamboo_base capability_check bamboo4WD_V4_YBStm32
    python3 -m bamboo_base.capability_check bamboo4WD_V4_YBStm32
"""
import os
import sys

# python3-yaml est une dependance dure de ros2launch : toujours presente la ou ce module
# tourne. hw_resolve.py, lui, s'interdit toute dependance parce qu'il sert aussi hors ROS.
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

# Convention de nommage des modules de controleur, voulue par l'utilisateur :
# "controler" avec UN SEUL l, et la casse du nom de carte preservee (YBStm32v3, WSEsp32...).
CONTROLER_PREFIX = "bamboo_controler_"


def controlerPackage(controller):
    """Nom du paquet ROS qui porte le controleur `controller`."""
    return CONTROLER_PREFIX + controller


def loadCapabilities(controller):
    """Lit le capabilities.yaml d'un controleur. Echoue en nommant ce qui manque."""
    package = controlerPackage(controller)
    try:
        share = get_package_share_directory(package)
    except PackageNotFoundError:
        raise RuntimeError(
            "controleur '%s' : paquet '%s' introuvable. Soit le nom est mal orthographie "
            "dans servo_axes (casse comprise), soit le module n'est pas construit "
            "(colcon build --packages-select %s)." % (controller, package, package))

    path = os.path.join(share, "config", "capabilities.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(
            "controleur '%s' : %s absent -- un module de controleur DOIT declarer ses "
            "capacites, sinon rien ne peut etre verifie avant l'actuation." % (
                controller, path))

    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    if "capabilities" not in doc:
        raise RuntimeError(
            "controleur '%s' : cle 'capabilities' absente de %s." % (controller, path))
    return doc


def loadRobot(robot):
    """Lit le fichier canonique du robot et aplatit ses ros__parameters."""
    path = os.path.join(
        get_package_share_directory("bamboo_base"), "config", "robots", robot + ".yaml")
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    params = {}
    for node_key in doc:
        params.update((doc[node_key] or {}).get("ros__parameters", {}) or {})
    return path, params


def checkServoAxes(robot):
    """Verifie chaque axe de servo_axes contre les capacites de son controleur.

    Renvoie la liste des lignes de compte rendu (une par axe). Leve RuntimeError des le
    premier axe invalide : on ne demarre pas a moitie une machine dont un axe est mal
    declare.
    """
    path, params = loadRobot(robot)
    axes = params.get("servo_axes") or {}
    if not axes:
        # Legitime : un robot sans camera motorisee n'a pas d'axe. Ce n'est pas une erreur.
        return ["%s : aucun axe de servo declare -- rien a verifier." % robot]

    report = []
    cache = {}
    for axis in sorted(axes):
        spec = axes[axis] or {}
        controller = spec.get("controller")
        if not controller:
            raise RuntimeError(
                "axe '%s' de %s : cle 'controller' absente. Un axe doit dire QUELLE carte "
                "l'execute, sinon la consigne n'a pas de destinataire." % (axis, path))
        channel = spec.get("channel")
        if channel is None:
            raise RuntimeError(
                "axe '%s' de %s : cle 'channel' absente." % (axis, path))

        if controller not in cache:
            cache[controller] = loadCapabilities(controller)
        caps = cache[controller]["capabilities"]
        voies = int(caps.get("servo_pwm", 0) or 0)

        if voies <= 0:
            raise RuntimeError(
                "axe '%s' de %s : le controleur '%s' (%s) NE COMMANDE AUCUN SERVO PWM "
                "(servo_pwm: 0). Deplacer cet axe sur une carte qui en a, ou le retirer -- "
                "le laisser la ferait partir les consignes dans le vide AVEC un ACK." % (
                    axis, path, controller, cache[controller].get("board", "?")))
        if int(channel) < 1 or int(channel) > voies:
            raise RuntimeError(
                "axe '%s' de %s : voie %s hors des %d voies PWM du controleur '%s' "
                "(ids MAVLink 1..%d)." % (axis, path, channel, voies, controller, voies))

        status = cache[controller].get("status", "?")
        report.append(
            "axe %-6s -> %-12s voie %d/%d  [%s]%s" % (
                axis, controller, int(channel), voies, status,
                "  << NON VALIDE SUR ROBOT >>" if status != "validated" else ""))
    return report


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    robot = argv[0] if argv else "bamboo4WD_V4_YBStm32"
    try:
        for line in checkServoAxes(robot):
            print(line)
    except RuntimeError as exc:
        print("ECHEC de la verification de capacites : %s" % exc, file=sys.stderr)
        return 1
    print("capacites coherentes pour %s" % robot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
