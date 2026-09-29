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
  2. bornes servo derivees du fichier canonique du robot, POUSSE par son bringup
  3. chemins de montage passes en arguments    -> faits d'HOTE, donc gagnants

Le point 2 merite l'`OpaqueFunction` qu'il coute : la table `servo_axes` du fichier canonique
est le SEUL endroit du depot qui lie un axe a une carte et a ses butees, et elle est
IMBRIQUEE (`servo_axes.pan.min_deg`). Un `parameters=[fichier]` ne la verrait pas -- ROS
ignore en silence les cles qu'un noeud ne declare pas -- et `servocam_node` retomberait sur
ses defauts compiles. On resout donc la table ici, a l'ouverture du launch, et on injecte
`pan_min` / `pan_max` / ... : c'est ce qui protege reellement les SG90 de la butee
(a 180 deg, ~700 mA continus dans des pignons plastique).

Le meme convoi porte `aspect`, pour la meme raison de source unique : il vaut HAUTEUR sur
LARGEUR du cadrage, donc il se DERIVE de `camera_width` / `camera_height`, les deux memes
nombres dont `bamboo_video` fait ses caps gstreamer. Ecrit a la main dans le YAML du groupe,
il y avait fini a l'ENVERS (w/h), ce qui ELARGIT la zone morte du pan au lieu de la resserrer
-- une camera qui ne suit pas horizontalement sans qu'aucun seuil ne paraisse fautif.

Ce groupe ne depend d'AUCUN module de controleur : il publie /servo/cmd en nommant les axes
(`pan`, `tilt`) et tourne sans executeur -- c'est ce qui rend le banc V6.3 possible robot
eteint.

Depuis E4, ce nom de sortie est un ARGUMENT (`servo_cmd_topic`), de defaut `/servo/cmd` donc
sans changement : le nom LOGIQUE appartient au noeud, le nom REEL au robot qui l'embarque. Le
remappage se pose ici, sur le ComposableNode, parce qu'un launch inclus ne peut pas recevoir
de `remappings=` de l'exterieur. Le meme nom doit etre pousse au module de controleur qui
ECOUTE, sinon la camera ne bouge plus sans qu'aucun des deux noeuds ne se plaigne.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from bamboo_base.robot_config import resolveRobotConfig  # ou vit la config d'un robot
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Axes attendus par servocam_node, et prefixe de parametre correspondant.
_AXES = ("pan", "tilt")


def _servoLimits(robot, config_path=""):
    """Derive course des servos ET aspect du cadrage depuis le fichier canonique du robot.

    `config_path` est POUSSE par le bringup du robot (E2) : ce module est generique et n'a
    pas a savoir ou vit la configuration d'une machine. Vide, resolveRobotConfig cherche,
    dans l'ordre documente la-bas, et echoue en NOMMANT les chemins essayes.

    Echoue en NOMMANT l'axe ou la cle manquante, jamais en silence : un axe dont les bornes
    seraient absentes se traduirait par un retour aux defauts compiles, donc par une butee
    possible -- exactement le piege du chantier 1 ou une capacite absente etait decouverte
    a l'execution.
    """
    path = resolveRobotConfig(robot, config_path)
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
        # Les SUFFIXES sont les noms de parametres du prototype (`--pan-min`, `--tilt-max`,
        # `--pan-home`) et non des noms en `_deg` : un essai du banc se transpose ainsi au
        # ROS sans retraduction. Le renommage doit rester SIMULTANE avec `servocam_node.cpp`
        # et `videotracking.yaml` -- un parametre pousse sous un nom que le noeud ne declare
        # pas est ignore EN SILENCE, et le noeud retombe sur son defaut compile.
        for key, suffix in (("min_deg", "min"),
                            ("max_deg", "max"),
                            ("rest_deg", "home")):
            if key not in spec:
                raise RuntimeError(
                    "axe '%s' : cle '%s' absente dans %s" % (axis, key, path))
            out["%s_%s" % (axis, suffix)] = float(spec[key])

    # aspect = HAUTEUR / LARGEUR, et le SENS compte : nx est normalise par w/2 et ny par h/2,
    # donc une zone morte CARREE A L'ECRAN demande un seuil de pan multiplie par h/w. Derive
    # des memes deux nombres que les caps gstreamer de bamboo_video : un seul cadrage declare
    # dans tout le depot, quelle que soit la resolution retenue.
    for key in ("camera_width", "camera_height"):
        if key not in params:
            raise RuntimeError(
                "'%s' absent de %s : impossible de deriver l'aspect du cadrage, donc la "
                "zone morte du pan" % (key, path))
    width = float(params["camera_width"])
    if width <= 0.0:
        raise RuntimeError("camera_width = %r dans %s : cadrage impossible" % (width, path))
    out["aspect"] = float(params["camera_height"]) / width
    return out


def _deadzone(group_cfg):
    """Zone morte de la loi, lue LA OU elle est declaree : `servocam_node` du YAML du groupe.

    Un parametre appartient a un noeud : `overlay_node` ne peut pas aller lire celui de
    `servocam_node` a l'execution. C'est donc le launch qui relaie l'UNIQUE valeur declaree
    aux deux noeuds. La redeclarer sous `overlay_node:` serait le pire des cas -- deux
    valeurs qui divergent en silence, et un reticule qui dessine une zone morte que la loi
    n'applique pas.

    Echoue en NOMMANT le fichier : sans zone morte, overlay_node retomberait sur son defaut
    compile et dessinerait une boite plausible mais fausse, ce qui est pire qu'aucune boite.
    """
    with open(group_cfg, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    params = (doc.get("servocam_node") or {}).get("ros__parameters") or {}
    if "deadzone" not in params:
        raise RuntimeError(
            "'deadzone' absent de servocam_node dans %s : le reticule d'overlay_node ne "
            "pourrait pas dire la zone morte reellement appliquee" % group_cfg)
    return float(params["deadzone"])


def _launchSetup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    robot_config = LaunchConfiguration("robot_config").perform(context)
    models_dir = LaunchConfiguration("models_dir").perform(context)
    faces_dir = LaunchConfiguration("faces_dir").perform(context)
    input_topic = LaunchConfiguration("input_topic").perform(context)
    container_name = LaunchConfiguration("container_name").perform(context)
    servo_cmd = LaunchConfiguration("servo_cmd_topic").perform(context)

    group_cfg = os.path.join(
        get_package_share_directory("bamboo_videotracking"),
        "config", "videotracking.yaml")
    limits = _servoLimits(robot, robot_config)

    hot = [group_cfg, {"models_dir": models_dir, "input_topic": input_topic}]
    recog = [group_cfg, {"models_dir": models_dir, "faces_dir": faces_dir}]
    servo = [group_cfg, limits]
    # HUD : le reticule doit suivre la MEME zone morte et le MEME `aspect` que la loi -- donc
    # la meme source, `servocam_node` du YAML de groupe pour l'une, le fichier canonique du
    # robot pour l'autre. Et le nom du topic servo est celui REELLEMENT en vigueur (le meme
    # que le remappage de servocam_node plus bas), sinon le HUD n'y verrait jamais passer une
    # consigne et la contre-epreuve de la zone morte serait impossible.
    overlay = hot + [{"deadzone": _deadzone(group_cfg),
                      "aspect": limits["aspect"],
                      "servo_cmd_topic": servo_cmd}]

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
                parameters=overlay,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bamboo_videotracking",
                plugin="bamboo_videotracking::ServoCamNode",
                name="servocam_node",
                parameters=servo,
                # La cle est le nom tel que le noeud le cree, donc ABSOLU
                # (servocam_node.cpp:111-112 publie "/servo/cmd") : une cle relative ne
                # correspondrait a rien et le remappage serait ignore en silence.
                remappings=[("/servo/cmd", servo_cmd)],
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
    models = ("/root/linorobot2_ws/src/linorobot2_hardware/firmware/usbcam_bamboo/"
              "Bambou4WD_python/src/"
              "resources/Other/face_detection_model")
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            # PAS de default_value (E6) : un module GENERIQUE ne choisit pas un robot en
            # silence. Sans defaut, launch refuse et NOMME l'argument manquant ; avec un
            # defaut, un module lance seul chargeait la geometrie et les bornes servo d'un
            # AUTRE robot sans un mot -- donc des butees possibles sur les SG90.
            description="Nom du robot (sans .yaml), OBLIGATOIRE. Sert a nommer le fichier "
                        "canonique quand robot_config n'est pas pousse. C'est le bringup "
                        "du robot qui le fournit (ou docker/.env, ROBOT=)."),
        DeclareLaunchArgument(
            "robot_config", default_value="",
            description="Chemin COMPLET du fichier canonique du robot, pousse par son "
                        "bringup. Vide : recherche par convention (<robot>_base/config/, "
                        "puis bamboo_base), qui echoue en nommant les chemins essayes."),
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
            "servo_cmd_topic", default_value="/servo/cmd",
            description="Nom REEL des consignes de servo produites par ce groupe. Defaut = "
                        "le nom logique, donc comportement inchange. Pousse par le bringup "
                        "du robot, qui doit pousser le MEME nom au module de controleur."),
        DeclareLaunchArgument(
            "container_name", default_value="videotracking_container",
            description="Nom du process de composition (component_container_mt)."),
        OpaqueFunction(function=_launchSetup),
    ])
