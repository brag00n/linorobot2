"""Lance le driver ESP32 MAVLink seul (test L5 lecture seule / L6 actuation).

Deux fichiers de parametres, dans cet ORDRE, et l'ordre est le contrat :
  1. config/esp32_driver.yaml            -> transport et frames (faits d'hote)
  2. config/robots/<robot>.yaml          -> geometrie et motorisation (source UNIQUE)
Le second gagne en cas de doublon : la source unique ne peut pas etre contredite par un
fichier local. Le driver ne porte aucun defaut geometrique -> si ce fichier manque ou est
incomplet, le noeud echoue en nommant le parametre absent, au lieu de rouler faux.

Par defaut enable_cmd_vel reste sur la valeur du YAML (false = lecture seule). Pour
l'actuation au L6 : enable_cmd_vel:=true (roues surelevees obligatoires).
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("bamboo_base"), "config", "esp32_driver.yaml")

    robot = LaunchConfiguration("robot")
    canonical = PathJoinSubstitution([
        FindPackageShare("bamboo_base"), "config", "robots", [robot, ".yaml"]])

    enable_cmd_vel = LaunchConfiguration("enable_cmd_vel")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_WSEsp32",
            description="Nom du fichier de source unique dans config/robots/ (sans .yaml)."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="true = actuation (L6, roues surelevees) ; false = lecture seule (L5)."),
        Node(
            package="bamboo_base",
            executable="esp32_mavlink_driver",
            name="esp32_mavlink_driver",
            output="screen",
            parameters=[cfg, canonical, {"enable_cmd_vel": enable_cmd_vel}],
        ),
    ])
