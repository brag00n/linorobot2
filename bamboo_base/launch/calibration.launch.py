"""Lance bamboo_calibration si le robot l'autorise (`calibration_enabled` du canonique).

Parametres, dans cet ORDRE (le dernier l'emporte) :
  1. fichier canonique du robot   -> geometrie, variantes calib.*, imu_gz_sign (bloc /**)
  2. config/calibration.yaml      -> la procedure (paliers, seuils)
  3. robot, output_dir            -> calcules ici

`output_dir` = le dossier SOURCE du fichier canonique (lien suivi : en --symlink-install
le fichier de share/ pointe vers src/), pour que le candidat atterrisse a cote du fichier
ou il devra etre recopie. Le depot est monte en ecriture dans le conteneur.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from bamboo_base.robot_config import resolveRobotConfig
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    canonical = resolveRobotConfig(robot, LaunchConfiguration("robot_config").perform(context))
    with open(canonical, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    params = {}
    for key in doc:
        params.update((doc[key] or {}).get("ros__parameters", {}) or {})
    if not params.get("calibration_enabled", False):
        return [LogInfo(msg="calibration : desactivee pour %s (calibration_enabled=false)"
                            % robot)]
    out = os.path.dirname(os.path.realpath(canonical))
    # Consigne vers le MULTIPLEXEUR (cmd_vel_calib du wiring `control`), sauf argument explicite.
    from bamboo_base.layout import loadWiring
    wiring, _notes = loadWiring(os.path.join(out, "layout.yaml"))
    cmd_topic = LaunchConfiguration("cmd_vel_topic").perform(context) or \
        wiring["control"]["cmd_vel_calib"]
    procedure = os.path.join(get_package_share_directory("bamboo_base"), "config",
                             "calibration.yaml")
    return [
        LogInfo(msg="calibration : %s, candidats dans %s" % (robot, out)),
        Node(package="bamboo_base", executable="calibration_node", name="bamboo_calibration",
             parameters=[canonical, procedure, {"robot": robot, "output_dir": out}],
             remappings=[("cmd_vel", cmd_topic)],
             output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", description="Nom du robot, OBLIGATOIRE."),
        DeclareLaunchArgument("robot_config", default_value="",
                              description="Chemin du fichier canonique (vide : convention)."),
        DeclareLaunchArgument("cmd_vel_topic", default_value="",
                              description="Consigne publiee pendant les paliers (vide : "
                                          "cmd_vel_calib du wiring, vers le multiplexeur)."),
        OpaqueFunction(function=_setup),
    ])
