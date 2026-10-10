"""Lance le lidar DU ROBOT, parametre par son paquet `<robot>_base` (lot NAV).

Sources, dans cet ordre :
  1. le fichier canonique du robot (resolveRobotConfig), bloc `lidar:` : modele, port, debit,
     trame, sens, recadrage, portee -- et bloc `sensors:` pour savoir s'il y a un lidar ;
  2. `<robot>_base/config/layout.yaml`, section `wiring/lidar` : le nom REEL du sujet du scan
     (bamboo_base.layout.loadWiring).

Aucune trame statique ici : la pose du capteur est dans l'URDF (lidar.pose_* du canonique,
lue par description.launch.py). Le driver d'origine en publiait une (base_link -> base_laser)
qui aurait concurrence l'URDF.

Proprietaire UNIQUE du port du lidar : conteneur driver.lidarD500, lance a la demande.
"""
import os

import yaml
from bamboo_base.layout import loadWiring
from bamboo_base.robot_config import resolveRobotConfig
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Modeles connus -> (paquet, executable) du driver. Une famille de plus = une ligne ici.
_DRIVERS = {
    "LDLiDAR_LD19": ("ldlidar_stl_ros2", "ldlidar_stl_ros2_node"),
    "LDLiDAR_LD06": ("ldlidar_stl_ros2", "ldlidar_stl_ros2_node"),
    "LDLiDAR_STL27L": ("ldlidar_stl_ros2", "ldlidar_stl_ros2_node"),
}


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    canonical = resolveRobotConfig(robot, LaunchConfiguration("robot_config").perform(context))
    with open(canonical, "r", encoding="utf-8") as fh:
        params = (yaml.safe_load(fh) or {}).get("/**", {}).get("ros__parameters", {}) or {}
    if not (params.get("sensors") or {}).get("lidar"):
        return [LogInfo(msg="lidar : %s n'a pas de lidar (sensors.lidar vide) -- rien a "
                            "lancer." % robot)]
    lid = params.get("lidar") or {}
    model = str(lid.get("model", ""))
    if model not in _DRIVERS:
        raise RuntimeError("lidar : modele '%s' inconnu (connus : %s). Le declarer dans "
                           "bamboo_lidar/launch/lidar.launch.py." % (model, ", ".join(_DRIVERS)))
    layout = os.path.join(os.path.dirname(os.path.realpath(canonical)), "layout.yaml")
    wiring, notes = loadWiring(layout)
    scan = wiring["lidar"]["scan"]
    pkg, exe = _DRIVERS[model]
    driver_params = {
        "product_name": model,
        "topic_name": scan,
        "frame_id": str(lid.get("frame", "laser")),
        "port_name": str(lid.get("port", "/dev/lidar")),
        "port_baudrate": int(lid.get("baud", 230400)),
        "laser_scan_dir": bool(lid.get("scan_ccw", True)),
        "enable_angle_crop_func": bool(lid.get("crop", False)),
        "angle_crop_min": float(lid.get("crop_min_deg", 135.0)),
        "angle_crop_max": float(lid.get("crop_max_deg", 225.0)),
    }
    return [LogInfo(msg=n) for n in notes] + [
        LogInfo(msg="lidar : %s (%s) sur %s -> %s, trame %s" % (
            model, robot, driver_params["port_name"], scan, driver_params["frame_id"])),
        Node(package=pkg, executable=exe, name="lidar_driver", output="screen",
             parameters=[driver_params]),
        Node(package="bamboo_lidar", executable="scan_monitor", name="scan_monitor",
             output="screen", parameters=[{"scan_topic": scan}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", description="Nom du robot, OBLIGATOIRE."),
        DeclareLaunchArgument("robot_config", default_value="",
                              description="Chemin du fichier canonique (vide : convention)."),
        OpaqueFunction(function=_setup),
    ])
