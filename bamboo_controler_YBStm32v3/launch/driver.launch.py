"""Lance le driver de la carte Yahboom ROS Control Board V3.0 (module YBStm32v3).

Deux fichiers de parametres, dans cet ORDRE, et l'ordre EST le contrat -- meme contrat
que bamboo_base/launch/driver.launch.py, recopie deliberement plutot qu'importe :
  1. <ce paquet>/config/stm32_driver.yaml       -> transport et frames (faits d'HOTE)
  2. le fichier canonique du robot, POUSSE en argument -> geometrie, PID, servo_axes
Le second gagne en cas de doublon : la source unique ne peut pas etre contredite par un
fichier local. Le driver ne porte AUCUN defaut physique -> si le fichier canonique manque
ou est incomplet, le noeud echoue en NOMMANT le parametre absent, au lieu de rouler faux.

Le fichier canonique n'est pas cherche ici : il decrit un ROBOT, pas une CARTE, et depuis
E2 c'est le paquet du robot qui le porte et le POUSSE par l'argument `robot_config`. Ce
module reste ignorant du robot -- c'est ce qui permet de le reutiliser tel quel sur une
autre machine. Le defaut de l'argument pointe encore config/robots/ de bamboo_base, pour
les robots qui n'ont pas encore leur propre paquet.

enable_cmd_vel reste sur la valeur du YAML (false = moteurs muets) sauf surcharge
explicite. Les SERVOS ne sont pas concernes par ce verrou : ils repondent toujours.

NOMS DE TOPICS : LOGIQUES ICI, REELS CHEZ LE ROBOT (E4)
-------------------------------------------------------
Le noeud s'abonne a des noms RELATIFS -- `cmd_vel` et `servo/cmd` -- et ce launch les expose
en arguments dont le defaut est ce meme nom : rien ne change tant que personne ne les pousse.
C'est le bringup du robot qui decide du nom REEL, parce que le cablage est un fait de MACHINE
et non de carte : ce robot ecoute la manette, un autre ecouterait un module de navigation, et
aucun des deux ne demande de toucher au driver.

Le remappage se fait ICI et pas depuis le bringup : `remappings=` appartient au Node, et un
launch inclus ne peut pas en recevoir de l'exterieur. D'ou le detour par des arguments.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bamboo_base.robot_config import resolveRobotConfig

import os


def _launchSetup(context, *args, **kwargs):
    """Resout le fichier canonique, puis declare le noeud.

    OpaqueFunction et non substitutions : `parameters=` veut un chemin de fichier QUI EXISTE,
    et choisir entre les emplacements possibles demande de connaitre la valeur de `robot`,
    donc le contexte. resolveRobotConfig echoue en nommant les chemins essayes, plutot que
    de laisser rclcpp annoncer un parametre absent sans dire quel fichier il n'a pas trouve.
    """
    robot = LaunchConfiguration("robot").perform(context)
    pushed = LaunchConfiguration("robot_config").perform(context)
    canonical = resolveRobotConfig(robot, pushed)

    cfg = os.path.join(
        get_package_share_directory("bamboo_controler_YBStm32v3"),
        "config", "stm32_driver.yaml")

    cmd_vel = LaunchConfiguration("cmd_vel_topic").perform(context)
    servo_cmd = LaunchConfiguration("servo_cmd_topic").perform(context)

    return [
        LogInfo(msg="fichier canonique du robot : " + canonical),
        LogInfo(msg="cablage : cmd_vel -> %s, servo/cmd -> %s" % (cmd_vel, servo_cmd)),
        Node(
            package="bamboo_controler_YBStm32v3",
            executable="stm32_mavlink_driver",
            name="stm32_mavlink_driver",
            output="screen",
            parameters=[cfg, canonical,
                        {"enable_cmd_vel": LaunchConfiguration("enable_cmd_vel")}],
            # Les deux cles sont les noms tels que le noeud les cree
            # (stm32_mavlink_driver.py:812 pour cmd_vel, :345 pour servo/cmd) : un remappage
            # dont la cle ne correspond a aucune souscription est ignore EN SILENCE, donc la
            # cle doit rester le nom relatif, meme quand la valeur est absolue.
            remappings=[("cmd_vel", cmd_vel), ("servo/cmd", servo_cmd)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_YBStm32",
            description="Nom du robot (sans .yaml). Sert a nommer le fichier canonique "
                        "quand robot_config n'est pas pousse. Propage depuis docker/.env "
                        "(ROBOT=)."),
        DeclareLaunchArgument(
            "robot_config", default_value="",
            description="Chemin COMPLET du fichier canonique du robot, pousse par son "
                        "bringup. Vide : recherche par convention (<robot>_base/config/, "
                        "puis bamboo_base), qui echoue en nommant les chemins essayes."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="true = actuation MOTEUR (roues surelevees obligatoires, apres "
                        "T2/T3/T5 de bambooSTM32YB) ; false = moteurs muets. Les servos "
                        "repondent dans les deux cas."),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="cmd_vel",
            description="Nom REEL de la consigne de deplacement ecoutee par la carte. "
                        "Defaut = le nom logique, donc comportement inchange. Pousse par le "
                        "bringup du robot (section wiring de son layout)."),
        DeclareLaunchArgument(
            "servo_cmd_topic", default_value="/servo/cmd",
            description="Nom REEL des consignes de servo ecoutees par la carte. Doit valoir "
                        "le meme nom que celui publie par le groupe tracking, sinon la "
                        "camera ne bouge plus et aucun des deux noeuds ne se plaint."),
        OpaqueFunction(function=_launchSetup),
    ])
