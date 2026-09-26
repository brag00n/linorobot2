"""Controleur WaveShare General Driver (ESP32-WROOM-32) : ENVELOPPE, pas un driver.

Ce module ne porte AUCUN code de driver, et c'est deliberement une dette assumee :
`esp32_mavlink_driver.py` reste dans `bamboo_base`, ou le chantier 1 -- en pause, et
partiellement execute -- l'a mis. Le deplacer casserait un plan approuve et des services
compose deja valides sur robot (`driver.real`). Ce launch se contente donc d'inclure
`bamboo_base/launch/driver.launch.py`.

Ce que l'enveloppe apporte malgre tout, et qui n'est pas cosmetique :
  * une adresse UNIFORME -- tout robot inclut `bamboo_controler_<carte>/driver.launch.py`,
    sans savoir lequel des modules porte son code ;
  * un `capabilities.yaml` que `bamboo_base.capability_check` sait lire -> un axe de
    `servo_axes` qui viserait cette carte fait desormais ECHOUER LE BRINGUP en nommant
    l'axe, au lieu d'envoyer des consignes que la carte refuse (UNSUPPORTED) avec un ACK.

Quand BambooWS reprendra, le driver descendra ici et cette enveloppe deviendra le module
complet ; les robots qui l'incluent n'auront pas une ligne a changer.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_WSEsp32",
            description="Nom du fichier de source unique dans "
                        "bamboo_base/config/robots/ (sans .yaml)."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="true = actuation moteur (roues surelevees obligatoires) ; "
                        "false = lecture seule. Defaut false, comme partout."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("bamboo_base"), "launch", "driver.launch.py"])),
            launch_arguments={
                "robot": LaunchConfiguration("robot"),
                "enable_cmd_vel": LaunchConfiguration("enable_cmd_vel"),
            }.items(),
        ),
    ])
