"""Groupe DIAGNOSTICS : diagnostic_aggregator + observateur bamboo_health.

Lance par le service compose `bamboo_diagnostics`, dans son IMAGE DEDIEE (stage `diagnostics`
du Dockerfile) : c'est la seule image qui porte diagnostic_aggregator.

    ros2 launch bamboo_diagnostics diagnostics.launch.py robot:=bamboo4WD_V4_YBStm32

L'ARBRE DEPEND DU ROBOT (lot BASCULE) : config/aggregator.yaml porte la partie commune, et
config/aggregator_<robot>.yaml ce qui n'existe que sur ce robot -- le module de SON driver de
carte, et le composant servocam_node sur un robot a nacelle. Les deux sont empiles, dans cet
ordre ; ROS fusionne leurs parametres imbriques.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launchSetup(context, *args, **kwargs):
    share = get_package_share_directory("bamboo_diagnostics")
    robot = LaunchConfiguration("robot").perform(context)
    common = os.path.join(share, "config", "aggregator.yaml")
    own = os.path.join(share, "config", "aggregator_%s.yaml" % robot)
    # ECHEC NOMME plutot qu'un arbre incomplet : sans sa partie propre, l'arbre n'aurait pas
    # de module driver, et un robot dont la carte ne repond plus paraitrait sain.
    if not os.path.isfile(own):
        raise RuntimeError(
            "aucune partie d'arbre pour le robot '%s' : %s absent. Robots connus : %s"
            % (robot, own, ", ".join(sorted(
                f[len("aggregator_"):-len(".yaml")]
                for f in os.listdir(os.path.dirname(own))
                if f.startswith("aggregator_") and f.endswith(".yaml")))))
    return [
        LogInfo(msg="diagnostics : arbre de %s (commun + %s)" % (robot, os.path.basename(own))),
        # Le nom `analyzers` n'est pas un choix : c'est celui sous lequel l'agregateur lit ses
        # parametres, et la cle de tete des fichiers aggregator*.yaml doit le reprendre.
        Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            name="analyzers",
            output="screen",
            parameters=[common, own],
        ),
        Node(
            package="bamboo_diagnostics",
            executable="health_node",
            name="bamboo_health",
            output="screen",
            parameters=[os.path.join(share, "config", "health.yaml")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            # PAS de defaut, comme les autres groupes : un module generique ne choisit pas un
            # robot en silence. Le service compose le fournit (ROBOT= de docker/.env).
            description="Nom du robot (sans .yaml), OBLIGATOIRE : il choisit la partie propre "
                        "de l'arbre de diagnostic (config/aggregator_<robot>.yaml)."),
        OpaqueFunction(function=_launchSetup),
    ])
