"""Lance le groupe manette : joy_linux_node + bamboo_teleop.

NOM DU FICHIER IMPOSE : les deux paquets de bringup incluent `control.launch.py` par ce
nom exact (`bamboo4WD_V4_YBStm32_base/launch/bringup.launch.py:161`). Le renommer casse
les deux bringup sans erreur de compilation -- l'echec n'apparait qu'au lancement.

CE GROUPE NE COMMANDE AUCUN MOTEUR DE TRACTION, c'est un choix de perimetre ecrit dans
`package.xml` et dans `teleop_node.py`. Il produit deux choses : des commandes de MODE
pour le groupe videotracking, et des pas relatifs sur les axes servo NOMMES.

Parametres, dans cet ORDRE, et l'ordre est le contrat (meme convention que
`bamboo_video/launch/video.launch.py`) :
  1. config/joy_bamboo.yaml  -> table des index et cadences (faits d'HOTE : ils dependent
                                de la manette et du mode d'appairage, pas du robot)
  2. arguments de launch     -> peripherique, car c'est un fait de machine

Le fichier canonique du robot n'est PAS une source de parametres ici -- il est lu
uniquement pour VERIFIER que les noms d'axes vises existent (voir _checkAxes). La table
`servo_axes` reste le seul endroit qui lie un nom d'axe a une carte et a une voie.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _configPath():
    return os.path.join(
        get_package_share_directory("bamboo_control"), "config", "joy_bamboo.yaml")


def _readParams(path, node_key):
    """Extrait le bloc ros__parameters d'un noeud dans un YAML de parametres ROS."""
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    return ((doc.get(node_key) or {}).get("ros__parameters", {}) or {})


def _checkAxes(context, *args, **kwargs):
    """Confronte les noms d'axes vises par la manette a la table `servo_axes` du robot.

    Motif : un nom d'axe errone ne casse RIEN au demarrage -- le noeud publie, le service
    repond `success: false`, et le seul symptome est une camera qui ne bouge pas au stick.
    On prefere le dire au lancement, en nommant l'axe et en listant ceux qui existent.

    Distinction volontaire entre les deux cas d'echec :
      * `servo_axes` ABSENT -> simple avertissement. Un robot sans servo peut legitimement
        porter une manette (modes seuls) ; les nudges seront refuses, ce qui est correct.
      * `servo_axes` PRESENT mais le nom absent -> ERREUR. C'est une faute de frappe, et
        la table prouve qu'on attendait des servos.
    Lecture de fichiers UNIQUEMENT : rien n'ouvre le port serie, mono-proprietaire.
    """
    robot = LaunchConfiguration("robot").perform(context)
    cfg = _readParams(_configPath(), "bamboo_teleop")
    wanted = [cfg.get("pan_axis_name", "pan"), cfg.get("tilt_axis_name", "tilt")]

    canonical = os.path.join(
        get_package_share_directory("bamboo_base"), "config", "robots", robot + ".yaml")
    with open(canonical, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    params = {}
    for node_key in doc:
        params.update((doc[node_key] or {}).get("ros__parameters", {}) or {})
    axes = params.get("servo_axes")

    if not axes:
        return [LogInfo(msg="manette : ATTENTION -- aucun `servo_axes` dans %s.yaml, les "
                            "nudges du stick droit seront refuses (modes seuls)." % robot)]
    missing = [name for name in wanted if name not in axes]
    if missing:
        raise RuntimeError(
            "manette : l'axe %s vise par joy_bamboo.yaml n'existe pas dans servo_axes de "
            "%s.yaml (axes declares : %s). Corriger pan_axis_name/tilt_axis_name ou la "
            "table servo_axes -- ne PAS contourner : un nom faux rend le stick muet sans "
            "aucune erreur a l'execution."
            % (", ".join(missing), robot, ", ".join(sorted(axes))))
    return [LogInfo(msg="manette : axes %s resolus dans servo_axes de %s"
                        % (" et ".join(wanted), robot))]


def generate_launch_description():
    config = _configPath()

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_YBStm32",
            description="Nom du fichier canonique dans bamboo_base/config/robots/ (sans "
                        ".yaml). Lu ICI seulement pour verifier les noms d'axes."),
        DeclareLaunchArgument(
            "joy_dev", default_value="/dev/input/js0",
            description="Peripherique manette. Fait de MACHINE : visible dans le "
                        "conteneur par `privileged` + /dev:/dev. Le numero depend de "
                        "l'ordre d'appairage -- a relever a T9 de bambooSTM32YB."),

        OpaqueFunction(function=_checkAxes),

        # joy_linux et NON joy : les deux publient le meme type, avec des index d'axes
        # DIFFERENTS. Ils ne sont pas substituables sans re-relever la table.
        Node(
            package="joy_linux", executable="joy_linux_node", name="joy_linux_node",
            parameters=[config, {"dev": LaunchConfiguration("joy_dev")}],
            output="screen",
        ),
        Node(
            package="bamboo_control", executable="teleop_node", name="bamboo_teleop",
            parameters=[config],
            output="screen",
        ),
    ])
