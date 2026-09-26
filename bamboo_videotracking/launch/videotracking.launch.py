"""Lance le groupe bamboo_videotracking : 4 composables C++ + l'apprentissage Python.

PREMIER ComposableNodeContainer du depot, et c'est tout l'interet du lot : les quatre
etages du chemin chaud vivent dans UN SEUL processus (`component_container_mt`), donc la
`cv::Mat` decodee une fois circule PAR POINTEUR entre eux. Hors container, chaque saut
couterait une trame `rgb8` 640x480 = 921 Ko recopies -- la memoire partagee DDS etant
coupee par l'entrypoint (`FASTRTPS_DEFAULT_PROFILES_FILE=/root/.shm_off.xml`).
=> Si un `sensor_msgs/Image` apparait entre ces quatre noeuds, la composition n'a PAS pris
   et tout le gain du C++ est perdu (verification 3 du plan).

`face_train_node` reste DEHORS : rclpy, hors chemin temps reel, et un batch d'enrolement
qui bloquerait le container ferait tomber la cadence de tracking.

Parametres, dans cet ORDRE, et l'ordre est le contrat (meme convention que
`bamboo_base/launch/driver.launch.py`) :
  1. config/videotracking.yaml                 -> reglages du groupe (faits de groupe)
  2. bornes servo derivees de config/robots/<robot>.yaml de bamboo_base (source UNIQUE)
  3. chemins de montage passes en arguments    -> faits d'HOTE, donc gagnants

Le point 2 merite l'`OpaqueFunction` qu'il coute : la table `servo_axes` du fichier canonique
est le SEUL endroit du depot qui lie un axe a une carte et a ses butees, et elle est
IMBRIQUEE (`servo_axes.pan.min_deg`). Un `parameters=[fichier]` ne la verrait pas -- ROS
ignore en silence les cles qu'un noeud ne declare pas -- et `servocam_node` retomberait sur
ses defauts compiles. On resout donc la table ici, a l'ouverture du launch, et on injecte
`pan_min_deg` / `pan_max_deg` / ... : c'est ce qui protege reellement les SG90 de la butee
(a 180 deg, ~700 mA continus dans des pignons plastique).

Ce groupe ne depend d'AUCUN module de controleur : il publie /servo/cmd en nommant les axes
(`pan`, `tilt`) et tourne sans executeur -- c'est ce qui rend le banc V6.3 possible robot
eteint.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Axes attendus par servocam_node, et prefixe de parametre correspondant.
_AXES = ("pan", "tilt")


def _servoLimits(robot):
    """Derive les parametres de course de servocam_node depuis la table `servo_axes`.

    Echoue en NOMMANT l'axe ou la cle manquante, jamais en silence : un axe dont les bornes
    seraient absentes se traduirait par un retour aux defauts compiles, donc par une butee
    possible -- exactement le piege du chantier 1 ou une capacite absente etait decouverte
    a l'execution.
    """
    path = os.path.join(
        get_package_share_directory("bamboo_base"), "config", "robots", robot + ".yaml")
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}

    # Format ROS natif : /**: ros__parameters: ... (donc utilisable par `ros2 param load`).
    params = {}
    for node_key in doc:
        params.update((doc[node_key] or {}).get("ros__parameters", {}) or {})

    axes = params.get("servo_axes")
    if not axes:
        raise RuntimeError(
            "servo_axes absent de %s : impossible de borner la course des servos" % path)

    out = {}
    for axis in _AXES:
        spec = axes.get(axis)
        if not spec:
            raise RuntimeError(
                "axe '%s' absent de servo_axes dans %s" % (axis, path))
        for key, suffix in (("min_deg", "min_deg"),
                            ("max_deg", "max_deg"),
                            ("rest_deg", "home_deg")):
            if key not in spec:
                raise RuntimeError(
                    "axe '%s' : cle '%s' absente dans %s" % (axis, key, path))
            out["%s_%s" % (axis, suffix)] = float(spec[key])
    return out


def _launchSetup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    models_dir = LaunchConfiguration("models_dir").perform(context)
    faces_dir = LaunchConfiguration("faces_dir").perform(context)
    input_topic = LaunchConfiguration("input_topic").perform(context)
    container_name = LaunchConfiguration("container_name").perform(context)

    group_cfg = os.path.join(
        get_package_share_directory("bamboo_videotracking"),
        "config", "videotracking.yaml")
    limits = _servoLimits(robot)

    hot = [group_cfg, {"models_dir": models_dir, "input_topic": input_topic}]
    recog = [group_cfg, {"models_dir": models_dir, "faces_dir": faces_dir}]
    servo = [group_cfg, limits]

    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        # _mt : les 4 composables ont chacun leur callback (image, track, mode, timer) et
        # le container mono-thread les serialiserait derriere le decodage JPEG.
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="bamboo_videotracking",
                plugin="bamboo_videotracking::TrackingNode",
                name="tracking_node",
                parameters=hot,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bamboo_videotracking",
                plugin="bamboo_videotracking::FaceRecogNode",
                name="facerecog_node",
                parameters=recog,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bamboo_videotracking",
                plugin="bamboo_videotracking::OverlayNode",
                name="overlay_node",
                parameters=hot,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bamboo_videotracking",
                plugin="bamboo_videotracking::ServoCamNode",
                name="servocam_node",
                parameters=servo,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    trainer = Node(
        package="bamboo_videotracking",
        executable="face_train_node.py",
        name="face_train_node",
        output="screen",
        parameters=[
            group_cfg,
            {"faces_dir": faces_dir,
             # Le .onnx est le MEME que celui du chemin chaud : une galerie enrolee avec un
             # autre modele ne serait pas comparable, les cosinus n'auraient plus de sens.
             "sface_model": os.path.join(
                 models_dir, "face_recognition_sface_2021dec.onnx")},
        ],
    )

    return [container, trainer]


def generate_launch_description():
    # Les chemins de modeles et de visages sont des faits d'HOTE (bind-mounts du depot
    # firmware et volume nomme), pas de la physique du robot -> arguments de launch, pas
    # fichier canonique. Voir docker-compose.yaml.
    models = ("/root/linorobot2_hardware/firmware/usbcam_bamboo/Bambou4WD_python/src/"
              "resources/Other/face_detection_model")
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_YBStm32",
            description="Nom du fichier de source unique dans bamboo_base/config/robots/."),
        DeclareLaunchArgument(
            "models_dir", default_value=models,
            description="Repertoire des .onnx YuNet/SFace (monte depuis le depot firmware)."),
        DeclareLaunchArgument(
            "faces_dir", default_value="/root/data/faces",
            description="Jeu d'apprentissage et galerie (volume nomme, NON versionne)."),
        DeclareLaunchArgument(
            "input_topic", default_value="/video/raw/compressed",
            description="Flux brut publie par bamboo_video : ce groupe en est CLIENT."),
        DeclareLaunchArgument(
            "container_name", default_value="videotracking_container",
            description="Nom du process de composition (component_container_mt)."),
        OpaqueFunction(function=_launchSetup),
    ])
