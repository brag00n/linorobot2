"""Bringup du ROBOT BambooWS (bamboo4WD_V4_WSEsp32) : tous les noeuds de cette machine.

JUMEAU de bamboo4WD_V4_YBStm32_base/launch/bringup.launch.py depuis le lot BASCULE
(2026-10-02) : memes mecaniques (layout lu dans config/, fichier canonique POUSSE aux modules,
controle des capacites avant tout noeud), memes groupes, memes arguments. Un essai se
transpose donc d'un robot a l'autre sans retraduction. Ce qui differe est ce que la MACHINE
differe, et rien d'autre :

  * la carte : WaveShare General Driver (ESP32, CP2102, /dev/esp32), module
    bamboo_controler_WSEsp32 ;
  * AUCUN servo : la carte n'a pas de sortie PWM (MAV_CMD_DO_SET_SERVO -> UNSUPPORTED), le
    fichier canonique n'a donc pas de `servo_axes`, et le groupe tracking tourne en CAMERA
    FIXE (detection, reconnaissance, apprentissage, HUD ; servocam_node n'est pas charge).
    Pour la meme raison il n'y a ni section `wiring` dans le layout, ni argument de cablage
    servo ici.

ROBOT EN PAUSE : l'attache moteur N20 de M1 est cassee, AUCUN ESSAI AU SOL avant reparation.
enable_cmd_vel reste false (argument ET fichier canonique).

Le lidar, le SLAM et la navigation restent des services compose distincts (slam.real,
navigation.real), hors de ce bringup comme avant.

Exemples :
  ros2 launch bamboo4WD_V4_WSEsp32_base bringup.launch.py
  ros2 launch bamboo4WD_V4_WSEsp32_base bringup.launch.py enable_tracking:=true
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from bamboo_base.capability_check import checkServoAxes
from bamboo_base.layout import layoutDefaults

_PKG = "bamboo4WD_V4_WSEsp32_base"


def _include(package, launch_file, arguments, flag):
    """Inclusion conditionnelle d'un launch de groupe (cf. le jumeau STM32 : le chemin est
    une SUBSTITUTION, donc un groupe desactive dont le paquet manque ne fait rien echouer)."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])),
        launch_arguments=arguments.items(),
        condition=IfCondition(LaunchConfiguration(flag)),
    )


def _resolveLayout(context, *args, **kwargs):
    """Donne aux arguments laisses vides le defaut ecrit dans config/layout.yaml."""
    path = LaunchConfiguration("layout_file").perform(context)
    flags, notes = layoutDefaults(path)
    actions = [LogInfo(msg="layout : " + line) for line in notes]
    for flag, value in sorted(flags.items()):
        # `.get` et non LaunchConfiguration(flag).perform() : layoutDefaults renvoie AUSSI les
        # noms de cablage servo par defaut, que ce bringup ne declare pas (aucun servo ici), et
        # perform() leve sur un argument non declare. Le poser sans effet est inoffensif.
        if context.launch_configurations.get(flag, "") == "":
            actions.append(SetLaunchConfiguration(flag, value))
    return actions


def _checkCapabilities(context, *args, **kwargs):
    """Confronte servo_axes aux capacites declarees : ici, il n'y a AUCUN axe, et c'est ce
    controle qui le dit au lancement plutot qu'un servo qui ne bougerait pas."""
    robot = LaunchConfiguration("robot").perform(context)
    config = LaunchConfiguration("robot_config").perform(context)
    return [LogInfo(msg="capacites : " + line)
            for line in checkServoAxes(robot, config)]


def generate_launch_description():
    robot = LaunchConfiguration("robot")
    robot_config = LaunchConfiguration("robot_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_WSEsp32",
            description="Nom du robot (sans .yaml). Propage depuis docker/.env (ROBOT=, ecrit "
                        "par docker/robot.sh) et transmis TEL QUEL a tous les groupes."),
        DeclareLaunchArgument(
            "robot_config",
            default_value=PathJoinSubstitution([
                FindPackageShare(_PKG), "config", [robot, ".yaml"]]),
            description="Chemin COMPLET du fichier canonique, pousse a tous les groupes."),
        DeclareLaunchArgument(
            "layout_file",
            default_value=PathJoinSubstitution([FindPackageShare(_PKG), "config", "layout.yaml"]),
            description="Composition de la machine (section groups)."),
        DeclareLaunchArgument(
            "enable_driver", default_value="",
            description="Driver de la carte WaveShare ESP32 (MAVLink sysid 2), LECTURE SEULE. "
                        "Proprietaire UNIQUE de /dev/esp32."),
        DeclareLaunchArgument(
            "enable_description", default_value="",
            description="robot_state_publisher (URDF + TF)."),
        DeclareLaunchArgument(
            "enable_video", default_value="",
            description="Groupe video : acquisition MJPG + mux + diffusion HTTP."),
        DeclareLaunchArgument(
            "enable_tracking", default_value="",
            description="Groupe tracking en CAMERA FIXE (aucun servo sur ce robot). FALSE par "
                        "defaut : il occupe plus de deux coeurs du RPi."),
        DeclareLaunchArgument(
            "enable_control", default_value="",
            description="Groupe manette (bamboo_control). Le stick droit n'a rien a bouger ; "
                        "la traction reste derriere ses deux verrous."),
        DeclareLaunchArgument(
            "joy_dev", default_value="/dev/input/js0",
            description="Peripherique manette (ordre d'appairage)."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="Actuation MOTEUR. Robot EN PAUSE (attache M1 cassee) : reste false, "
                        "aucun essai au sol."),
        DeclareLaunchArgument(
            "video_device", default_value="/dev/video0",
            description="Peripherique V4L2 de la camera de CE robot."),

        OpaqueFunction(function=_resolveLayout),
        OpaqueFunction(function=_checkCapabilities),

        # --- la carte -----------------------------------------------------------------
        _include("bamboo_controler_WSEsp32", "driver.launch.py",
                 {"robot": robot, "robot_config": robot_config,
                  "enable_cmd_vel": LaunchConfiguration("enable_cmd_vel")},
                 "enable_driver"),

        # --- URDF / TF ----------------------------------------------------------------
        _include("linorobot2_description", "description.launch.py",
                 {"robot": robot, "robot_config": robot_config,
                  "rviz": "false", "publish_joints": "false"},
                 "enable_description"),

        # --- groupe video -------------------------------------------------------------
        _include("bamboo_video", "video.launch.py",
                 {"robot": robot, "robot_config": robot_config,
                  "stream_mode": "mjpg",
                  "video_device": LaunchConfiguration("video_device")},
                 "enable_video"),

        # --- groupe tracking, CAMERA FIXE --------------------------------------------
        # Pas de servo_cmd_topic : le launch du groupe ne charge pas servocam_node quand le
        # fichier canonique n'a pas de servo_axes, donc personne ne publierait de consigne.
        _include("bamboo_videotracking", "videotracking.launch.py",
                 {"robot": robot, "robot_config": robot_config},
                 "enable_tracking"),

        # --- groupe manette -----------------------------------------------------------
        _include("bamboo_control", "control.launch.py",
                 {"robot": robot, "robot_config": robot_config,
                  "joy_dev": LaunchConfiguration("joy_dev")},
                 "enable_control"),
    ])
