"""Lance le driver ESP32 MAVLink de BambooWS (lecture seule par defaut).

Meme CONTRAT que bamboo_controler_YBStm32v3/launch/driver.launch.py, et c'est l'objet du lot
BASCULE : les deux robots se lancent par les memes arguments, donc un bringup, un service
compose ou un essai se transpose de l'un a l'autre sans retraduction.

Deux fichiers de parametres, dans cet ORDRE, et l'ordre est le contrat :
  1. config/esp32_driver.yaml       -> transport et frames (faits d'HOTE)
  2. le fichier canonique du robot  -> geometrie et motorisation (source UNIQUE)
Le second gagne en cas de doublon : la source unique ne peut pas etre contredite par un
fichier local. Le driver ne porte aucun defaut geometrique -> si ce fichier manque ou est
incomplet, le noeud echoue en nommant le parametre absent, au lieu de rouler faux.

Le fichier canonique etait ICI code en dur (bamboo_base/config/robots/), alors que le
resolveur commun en connaissait deja deux emplacements : il a demenage dans
bamboo4WD_V4_WSEsp32_base/config/ au lot BASCULE, comme celui de bambooSTM32YB a l'etape E2.
Le chemin est donc POUSSE par le bringup (`robot_config`), ou RESOLU par convention.

Pas d'argument `servo_cmd_topic`, contrairement au STM32 : cette carte n'a aucune sortie servo
(MAV_CMD_DO_SET_SERVO -> UNSUPPORTED), et le driver ne cree aucune interface servo. Une
capacite absente doit etre ABSENTE de l'interface, pas decouverte a l'execution.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bamboo_base.robot_config import resolveRobotConfig


def _launchSetup(context, *args, **kwargs):
    """Resout le fichier canonique, puis declare le noeud (meme raison qu'au STM32 :
    `parameters=` veut un fichier QUI EXISTE, et le choisir demande la valeur de `robot`)."""
    robot = LaunchConfiguration("robot").perform(context)
    pushed = LaunchConfiguration("robot_config").perform(context)
    canonical = resolveRobotConfig(robot, pushed)

    cfg = os.path.join(
        get_package_share_directory("bamboo_base"), "config", "esp32_driver.yaml")
    cmd_vel = LaunchConfiguration("cmd_vel_topic").perform(context)

    return [
        LogInfo(msg="fichier canonique du robot : " + canonical),
        LogInfo(msg="cablage : cmd_vel -> %s (aucun servo sur cette carte)" % cmd_vel),
        Node(
            package="bamboo_base",
            executable="esp32_mavlink_driver",
            name="esp32_mavlink_driver",
            output="screen",
            parameters=[cfg, canonical,
                        {"enable_cmd_vel": LaunchConfiguration("enable_cmd_vel")}],
            # La cle est le nom tel que le noeud cree sa souscription (relatif, "cmd_vel") :
            # un remappage dont la cle ne correspond a rien est ignore EN SILENCE.
            remappings=[("cmd_vel", cmd_vel)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            # PAS de default_value, comme le STM32 depuis E6 : un module generique ne choisit
            # pas un robot en silence. C'est le bringup du robot, ou docker/.env (ROBOT=) via
            # le service compose, qui le fournit.
            description="Nom du robot (sans .yaml), OBLIGATOIRE."),
        DeclareLaunchArgument(
            "robot_config", default_value="",
            description="Chemin COMPLET du fichier canonique, pousse par le bringup. Vide : "
                        "recherche par convention (<robot>_base/config/, puis bamboo_base), "
                        "qui echoue en nommant les chemins essayes."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="true = actuation MOTEUR : le robot roule des qu'un producteur "
                        "publie cmd_vel. false par defaut ; le service driver.esp32 l'arme."),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="cmd_vel",
            description="Nom REEL de la consigne de deplacement ecoutee par la carte. "
                        "Defaut = le nom logique, donc comportement inchange."),
        OpaqueFunction(function=_launchSetup),
    ])
