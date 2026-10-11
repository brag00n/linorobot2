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
le scan vient de Gazebo. Le multiplexeur de bamboo_control (que le vrai robot lance dans son
conteneur bamboo_control) est alors lance ICI, sans quoi `cmd_vel_nav` n'atteindrait jamais
le `cmd_vel` que consomme le plugin Gazebo.
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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

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

    remap_tf = [("/tf", "tf"), ("/tf_static", "tf_static")]

    def comp(pkg, plugin, name, params, remaps=()):
        return ComposableNode(package=pkg, plugin=plugin, name=name, parameters=params,
                              remappings=remap_tf + list(remaps))
    # Nav2 en COMPOSITION : un seul processus, donc un seul participant DDS. En onze
    # processus separes (essai Nb du 2026-10-10), la decouverte et les echanges inter-
    # processus portaient la charge du RPi4 a 30 sur 4 coeurs, et l'EKF ratait sa cadence.
    # Plugins = ceux de nav2_bringup (use_composition) en Humble.
    nav2_container = ComposableNodeContainer(
        name="nav2_container", namespace="", package="rclcpp_components",
        executable="component_container_isolated", output="screen",
        # Le fichier Nav2 passe AUSSI au CONTENEUR (--params-file du processus), comme le fait
        # nav2_bringup : les costmaps sont des SOUS-NOEUDS crees par controller_server et
        # planner_server (/local_costmap/local_costmap...), et les `parameters` d'un
        # ComposableNode ne servent que le noeud qu'il nomme. Sans cela les deux costmaps
        # tournaient avec leurs DEFAUTS -- carte statique seule, AUCUNE source d'obstacles
        # (le lidar n'y entrait pas) : essai E2 du 2026-10-11, le robot a fonce dans un mur.
        parameters=[nav2],
        composable_node_descriptions=[
            comp("nav2_controller", "nav2_controller::ControllerServer", "controller_server",
                 [nav2], [("cmd_vel", smoothed)]),
            comp("nav2_smoother", "nav2_smoother::SmootherServer", "smoother_server", [nav2]),
            comp("nav2_planner", "nav2_planner::PlannerServer", "planner_server", [nav2]),
            comp("nav2_behaviors", "behavior_server::BehaviorServer", "behavior_server",
                 [nav2], [("cmd_vel", w["cmd_vel_nav"])]),
            comp("nav2_bt_navigator", "nav2_bt_navigator::BtNavigator", "bt_navigator",
                 [nav2]),
            comp("nav2_waypoint_follower", "nav2_waypoint_follower::WaypointFollower",
                 "waypoint_follower", [nav2]),
            comp("nav2_velocity_smoother", "nav2_velocity_smoother::VelocitySmoother",
                 "velocity_smoother", [nav2],
                 [("cmd_vel", smoothed), ("cmd_vel_smoothed", w["cmd_vel_nav"])]),
            comp("nav2_map_server", "nav2_map_server::MapServer", "map_server", [nav2],
                 [("map", w["map"])]),
            comp("nav2_amcl", "nav2_amcl::AmclNode", "amcl", [nav2], [("map", w["map"])]),
            comp("nav2_lifecycle_manager", "nav2_lifecycle_manager::LifecycleManager",
                 "lifecycle_manager_navigation",
                 [{"use_sim_time": use_sim_time, "autostart": False,
                   "node_names": _NAV_NODES, "bond_timeout": 0.0}]),
            comp("nav2_lifecycle_manager", "nav2_lifecycle_manager::LifecycleManager",
                 "lifecycle_manager_localization",
                 [{"use_sim_time": use_sim_time, "autostart": False,
                   "node_names": _LOC_NODES, "bond_timeout": 0.0}]),
        ])
    lifecycle = []
    nodes = [
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             output="screen", parameters=[ekf],
             remappings=[("odometry/filtered", w["odom"])]),
        nav2_container,
        Node(package="bamboo_navigation", executable="nav_supervisor", name="nav_supervisor",
             output="screen",
             parameters=[{"use_sim_time": use_sim_time, "maps_dir": maps_dir,
                          "slam_params_file": slam, "default_map": default_map,
                          "required_sensors": required or [""],
                          "scan_topic": w["scan"], "imu_topic": w["imu"],
                          "odom_topic": w["odom"], "map_topic": w["map"],
                          # pause de l'exploration auto quand la manette parle
                          "teleop_topic": wiring["control"]["cmd_vel_teleop"]}]),
    ]
    info = [LogInfo(msg=n) for n in notes] + [LogInfo(
        msg="navigation : %s%s, capteurs exiges %s, scan %s, odom %s, sortie %s, cartes %s"
            % (robot, " (SIMULATION)" if sim else "", required or "aucun", w["scan"],
               w["odom"], w["cmd_vel_nav"], maps_dir))]
    if not required or "lidar" not in required:
        info.append(LogInfo(msg="navigation : ATTENTION, %s n'a pas de lidar declare -- la "
                                "navigation sera declaree impossible (diagnostic)." % robot))
    if sim:
        control = wiring["control"]
        nodes.append(Node(
            package="bamboo_control", executable="cmd_vel_mux", name="cmd_vel_mux",
            output="screen",
            parameters=[os.path.join(get_package_share_directory("bamboo_control"), "config",
                                     "cmd_vel_mux.yaml"),
                        {"use_sim_time": True, "output_topic": control["cmd_vel"],
                         "topics.teleop.topic": control["cmd_vel_teleop"],
                         "topics.calibration.topic": control["cmd_vel_calib"],
                         "topics.navigation.topic": control["cmd_vel_nav"]}]))
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
