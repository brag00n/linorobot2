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

TRANSMET TOUT LE CONTRAT depuis le lot BASCULE (robot, robot_config, enable_cmd_vel,
cmd_vel_topic), comme bamboo_controler_YBStm32v3/launch/driver.launch.py. Avant, elle ne
relayait que `robot` et `enable_cmd_vel` : un bringup qui poussait le fichier canonique le
perdait EN SILENCE, et le driver retombait sur un chemin code en dur.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Les arguments relayes TELS QUELS au driver de bamboo_base. Une seule liste, pour qu'un
# argument ajoute la-bas ne soit pas oublie ici.
_RELAYED = ("robot", "robot_config", "enable_cmd_vel", "cmd_vel_topic")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            description="Nom du robot (sans .yaml), OBLIGATOIRE, comme pour le STM32."),
        DeclareLaunchArgument(
            "robot_config", default_value="",
            description="Chemin COMPLET du fichier canonique, pousse par le bringup."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="true = actuation moteur (roues surelevees obligatoires) ; "
                        "false = lecture seule. Defaut false, comme partout."),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="cmd_vel",
            description="Nom REEL de la consigne de deplacement ecoutee par la carte."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("bamboo_base"), "launch", "driver.launch.py"])),
            # Liste de PAIRES, jamais un dict (cf. bamboo_base/launch/drive.launch.py).
            launch_arguments=[(n, LaunchConfiguration(n)) for n in _RELAYED],
        ),
    ])
