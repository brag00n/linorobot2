"""Groupe DIAGNOSTICS : diagnostic_aggregator + observateur bamboo_health.

Lance par le service compose `bamboo_diagnostics`, dans son IMAGE DEDIEE (stage `diagnostics`
du Dockerfile) : c'est la seule image qui porte diagnostic_aggregator.

    ros2 launch bamboo_diagnostics diagnostics.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("bamboo_diagnostics")
    return LaunchDescription([
        # Le nom `analyzers` n'est pas un choix : c'est celui sous lequel l'agregateur lit ses
        # parametres, et la cle de tete de config/aggregator.yaml doit le reprendre.
        Node(
            package="diagnostic_aggregator",
            executable="aggregator_node",
            name="analyzers",
            output="screen",
            parameters=[os.path.join(share, "config", "aggregator.yaml")],
        ),
        Node(
            package="bamboo_diagnostics",
            executable="health_node",
            name="bamboo_health",
            output="screen",
            parameters=[os.path.join(share, "config", "health.yaml")],
        ),
    ])
