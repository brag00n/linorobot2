"""Navigation DU ROBOT : EKF + Nav2 + supervision (sous-modes exploration / navigation).

Tout vient du paquet `<robot>_base` (lot NAV) :
  - fichier canonique (resolveRobotConfig) : `sensors:` (capteurs exiges), `lidar:` (portee),
    `navigation:` (fichier Nav2, dossier des cartes, carte par defaut, bornes de vitesse) ;
  - `config/layout.yaml`, `wiring/navigation` : noms REELS des sujets ;
  - `config/<navigation.params_file>` : les parametres Nav2 du robot.
Les sujets et les bornes sont INJECTES dans des copies des fichiers (nav_params), ecrites
dans /tmp : aucun fichier de configuration ne porte un nom de sujet qui fasse foi.

Ce que ce launch NE lance PAS : slam_toolbox. C'est nav_supervisor qui le demarre en
sous-mode exploration et l'arrete ensuite (il n'a pas de cycle de vie). map_server et AMCL
sont lances INACTIFS (lifecycle_manager_localization sans autostart), le coeur de Nav2 aussi :
le superviseur active ce que le sous-mode demande.

Sortie de Nav2 : le controleur passe par le velocity_smoother, et les comportements (spin,
backup) publient directement -- les deux sur `cmd_vel_nav` du wiring, entree de plus faible
priorite du multiplexeur de bamboo_control. La manette garde toujours la main.

SIMULATION : sim:=true use_sim_time:=true avec linorobot2_gazebo (ekf:=false) : memes sujets,
le scan vient de Gazebo.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from bamboo_base.layout import loadWiring
from bamboo_base.robot_config import resolveRobotConfig
from bamboo_navigation import nav_params
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_NAV_NODES = ["controller_server", "smoother_server", "planner_server", "behavior_server",
              "bt_navigator", "waypoint_follower", "velocity_smoother"]
_LOC_NODES = ["map_server", "amcl"]


def _load(path):
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _dump(doc, tag):
    fd, path = tempfile.mkstemp(prefix="bamboo_%s_" % tag, suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as fh:
        yaml.safe_dump(doc, fh, sort_keys=False)
    return path


def _setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    sim = LaunchConfiguration("sim").perform(context).lower() in ("1", "true", "yes")
    use_sim_time = sim or LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "1", "true", "yes")
    canonical = resolveRobotConfig(robot, LaunchConfiguration("robot_config").perform(context))
    base_dir = os.path.dirname(os.path.realpath(canonical))      # <robot>_base/config (src)
    params = _load(canonical).get("/**", {}).get("ros__parameters", {}) or {}
    nav = params.get("navigation") or {}
    lidar = params.get("lidar") or {}
    required = nav_params.required_sensors(params.get("sensors"))
    wiring, notes = loadWiring(os.path.join(base_dir, "layout.yaml"))
    w = wiring["navigation"]

    share = get_package_share_directory("bamboo_navigation")
    nav2_file = os.path.join(base_dir, str(nav.get("params_file", "navigation.yaml")))
    nav2 = _dump(nav_params.patch_nav2(_load(nav2_file), w, nav, use_sim_time), "nav2")
    ekf = _dump(nav_params.patch_ekf(_load(os.path.join(share, "config", "ekf.yaml")), w,
                                     use_sim_time), "ekf")
    slam = _dump(nav_params.patch_slam(_load(os.path.join(share, "config", "slam.yaml")), w,
                                       lidar, use_sim_time), "slam")
    maps_dir = os.path.join(os.path.dirname(base_dir), str(nav.get("maps_dir", "maps")))
    default_map = LaunchConfiguration("map").perform(context) or str(nav.get("default_map", ""))
    smoothed = "/navigation/cmd_vel_raw"          # controleur -> smoother (interne)

    lifecycle = [
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": False,
                          "node_names": _NAV_NODES, "bond_timeout": 0.0}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": False,
                          "node_names": _LOC_NODES, "bond_timeout": 0.0}]),
    ]
    remap_tf = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nodes = [
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             output="screen", parameters=[ekf],
             remappings=[("odometry/filtered", w["odom"])]),
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[nav2], remappings=remap_tf + [("cmd_vel", smoothed)]),
        Node(package="nav2_smoother", executable="smoother_server", name="smoother_server",
             output="screen", parameters=[nav2], remappings=remap_tf),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[nav2], remappings=remap_tf),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=[nav2],
             remappings=remap_tf + [("cmd_vel", w["cmd_vel_nav"])]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             output="screen", parameters=[nav2], remappings=remap_tf),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             name="waypoint_follower", output="screen", parameters=[nav2],
             remappings=remap_tf),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen", parameters=[nav2],
             remappings=remap_tf + [("cmd_vel", smoothed),
                                    ("cmd_vel_smoothed", w["cmd_vel_nav"])]),
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[nav2],
             remappings=remap_tf + [("map", w["map"])]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[nav2], remappings=remap_tf + [("map", w["map"])]),
        Node(package="bamboo_navigation", executable="nav_supervisor", name="nav_supervisor",
             output="screen",
             parameters=[{"use_sim_time": use_sim_time, "maps_dir": maps_dir,
                          "slam_params_file": slam, "default_map": default_map,
                          "required_sensors": required or [""],
                          "scan_topic": w["scan"], "imu_topic": w["imu"],
                          "odom_topic": w["odom"], "map_topic": w["map"]}]),
    ]
    info = [LogInfo(msg=n) for n in notes] + [LogInfo(
        msg="navigation : %s%s, capteurs exiges %s, scan %s, odom %s, sortie %s, cartes %s"
            % (robot, " (SIMULATION)" if sim else "", required or "aucun", w["scan"],
               w["odom"], w["cmd_vel_nav"], maps_dir))]
    if not required or "lidar" not in required:
        info.append(LogInfo(msg="navigation : ATTENTION, %s n'a pas de lidar declare -- la "
                                "navigation sera declaree impossible (diagnostic)." % robot))
    return info + nodes + lifecycle


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", description="Nom du robot, OBLIGATOIRE."),
        DeclareLaunchArgument("robot_config", default_value="",
                              description="Chemin du fichier canonique (vide : convention)."),
        DeclareLaunchArgument("sim", default_value="false",
                              description="true = simulation (linorobot2_gazebo)."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value="",
                              description="Carte de la navigation (vide : canonique, puis "
                                          "la plus recente)."),
        OpaqueFunction(function=_setup),
    ])
