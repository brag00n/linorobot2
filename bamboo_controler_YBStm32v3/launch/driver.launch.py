"""Lance le driver de la carte Yahboom ROS Control Board V3.0 (module YBStm32v3).

Deux fichiers de parametres, dans cet ORDRE, et l'ordre EST le contrat -- meme contrat
que bamboo_base/launch/driver.launch.py, recopie deliberement plutot qu'importe :
  1. <ce paquet>/config/stm32_driver.yaml       -> transport et frames (faits d'HOTE)
  2. bamboo_base/config/robots/<robot>.yaml     -> geometrie, PID, servo_axes (source UNIQUE)
Le second gagne en cas de doublon : la source unique ne peut pas etre contredite par un
fichier local. Le driver ne porte AUCUN defaut physique -> si le fichier canonique manque
ou est incomplet, le noeud echoue en NOMMANT le parametre absent, au lieu de rouler faux.

Le fichier canonique est cherche dans bamboo_base et non ici : il decrit un ROBOT, pas une
CARTE. Un robot compose plusieurs controleurs, qui doivent tous lire la meme verite.

enable_cmd_vel reste sur la valeur du YAML (false = moteurs muets) sauf surcharge
explicite. Les SERVOS ne sont pas concernes par ce verrou : ils repondent toujours.
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
        get_package_share_directory("bamboo_controler_YBStm32v3"),
        "config", "stm32_driver.yaml")

    robot = LaunchConfiguration("robot")
    canonical = PathJoinSubstitution([
        FindPackageShare("bamboo_base"), "config", "robots", [robot, ".yaml"]])

    enable_cmd_vel = LaunchConfiguration("enable_cmd_vel")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_YBStm32",
            description="Nom du fichier de source unique dans bamboo_base/config/robots/ "
                        "(sans .yaml). Propage depuis docker/.env (ROBOT=)."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="true = actuation MOTEUR (roues surelevees obligatoires, apres "
                        "T2/T3/T5 de bambooSTM32YB) ; false = moteurs muets. Les servos "
                        "repondent dans les deux cas."),
        Node(
            package="bamboo_controler_YBStm32v3",
            executable="stm32_mavlink_driver",
            name="stm32_mavlink_driver",
            output="screen",
            parameters=[cfg, canonical, {"enable_cmd_vel": enable_cmd_vel}],
        ),
    ])
