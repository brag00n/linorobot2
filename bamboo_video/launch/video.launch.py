"""Lance le groupe bamboo_video : PROPRIETAIRE unique de la camera et de la diffusion.

AUTONOME PAR CONSTRUCTION, et c'est l'exigence explicite de la demande : ce launch ne
reference AUCUN noeud de bamboo_videotracking, ne declare aucun `depends_on`, et sert le
flux brut tant que personne ne s'est enregistre. Le sens de la dependance est a sens
unique : le tracking est CLIENT de ce groupe (il appelle /video/register_overlay), jamais
l'inverse.

Le `stream_mux_node` donne au navigateur UNE SEULE URL STABLE : `/video/stream/compressed`.
Elle porte le brut ou l'enrichi selon le registre -- le client externe ne change donc jamais
d'adresse selon que le tracking tourne ou non.

DEUX MODES, MUTUELLEMENT EXCLUSIFS, et c'est une contrainte MATERIELLE (UVC : /dev/video0
ne sert qu'UN format a la fois), pas un choix de conception :
  - `mjpg`  : gscam2 recopie le MJPG materiel SANS le decoder (0 conversion) ->
              /video/raw/compressed a ~30 fps, ~30 Ko/trame. LE TRACKING EST POSSIBLE.
  - `h264`  : on ne demarre PAS gscam2. MediaMTX (service compose `camera.h264`, HORS pile
              ROS) garde la camera et sert RTSP 8554 / HLS 8888 / WebRTC 8889 : meilleure
              qualite, charge quasi nulle, mais AUCUN TRACKING POSSIBLE -- aucune trame
              n'entre dans ROS. Ce launch le dit a voix haute au demarrage plutot que de
              publier un flux vide.
`camera.real` (v4l2 YUYV->rgb8) est ECARTE des deux : ~12 fps plafond et 4 coeurs satures,
il consommerait tout le budget du tracking. Les trois chemins partagent le port 8080.

Parametres, dans cet ORDRE, et l'ordre est le contrat (meme convention que
`bamboo_videotracking/launch/videotracking.launch.py`) :
  1. config/video.yaml                        -> reglages du groupe (topics, delais)
  2. resolution lue dans config/robots/<robot>.yaml de bamboo_base (source UNIQUE)
  3. arguments de launch (peripherique, port)  -> faits d'HOTE, donc gagnants
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_MODES = ("mjpg", "h264")


def _cameraSize(robot):
    """Lit `camera_width` / `camera_height` dans le fichier canonique du robot.

    Echoue en NOMMANT la cle manquante : une resolution absente ferait negocier a v4l2src
    un format arbitraire, et la loi de commande pan/tilt -- qui normalise par la demi-largeur
    -- travaillerait alors sur une geometrie differente de celle declaree.
    """
    path = os.path.join(
        get_package_share_directory("bamboo_base"), "config", "robots", robot + ".yaml")
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}

    params = {}
    for node_key in doc:
        params.update((doc[node_key] or {}).get("ros__parameters", {}) or {})

    out = {}
    for key in ("camera_width", "camera_height"):
        if key not in params:
            raise RuntimeError("cle '%s' absente de %s" % (key, path))
        out[key] = int(params[key])
    return out


def _launchSetup(context, *args, **kwargs):
    mode = LaunchConfiguration("stream_mode").perform(context).strip().lower()
    if mode not in _MODES:
        raise RuntimeError(
            "stream_mode='%s' inconnu : attendu %s" % (mode, " | ".join(_MODES)))

    robot = LaunchConfiguration("robot").perform(context)
    device = LaunchConfiguration("video_device").perform(context)
    framerate = LaunchConfiguration("framerate").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    port = LaunchConfiguration("port").perform(context)
    raw_topic = LaunchConfiguration("raw_topic").perform(context)
    out_topic = LaunchConfiguration("out_topic").perform(context)
    fps_metric = LaunchConfiguration("enable_fps_metric").perform(context).lower()

    group_cfg = os.path.join(
        get_package_share_directory("bamboo_video"), "config", "video.yaml")

    if mode == "h264":
        # Rien de ROS a demarrer : MediaMTX tient la camera. On le DIT au lieu de lancer une
        # chaine qui ne verrait jamais une trame -- un mux qui publie du vide ressemble a une
        # panne, alors que c'est le mode demande.
        return [LogInfo(msg=(
            "bamboo_video en mode h264 : la camera est servie par MediaMTX "
            "(service compose camera.h264, RTSP 8554 / HLS 8888 / WebRTC 8889). "
            "Aucune trame n'entre dans ROS -> LE TRACKING EST IMPOSSIBLE dans ce mode. "
            "Basculer stream_mode:=mjpg pour l'activer."))]

    size = _cameraSize(robot)

    # gscam2 en passthrough JPEG : recette EPROUVEE de camera_mjpg.launch.py, reprise telle
    # quelle. `image_encoding: jpeg` fait recopier le buffer MJPG materiel dans un
    # CompressedImage, donc ZERO decodage. Le noeud ne publie QUE image_raw/compressed (pas
    # de image_raw brut) : on le remappe sur le topic d'entree du groupe.
    gscam = Node(
        package="gscam2",
        executable="gscam_main",
        name="gscam_node",
        output="screen",
        parameters=[{
            "gscam_config": (
                "v4l2src device=" + device + " do-timestamp=true ! image/jpeg,width="
                + str(size["camera_width"]) + ",height=" + str(size["camera_height"])
                + ",framerate=" + framerate + "/1"),
            "image_encoding": "jpeg",
            "camera_name": "camera",
            "frame_id": frame_id,
            "sync_sink": False,  # ne pas bloquer sur l'horloge : on veut le debit brut
        }],
        remappings=[("image_raw/compressed", raw_topic)],
    )

    mux = Node(
        package="bamboo_video",
        executable="stream_mux_node",
        name="stream_mux_node",
        output="screen",
        parameters=[group_cfg, {"raw_topic": raw_topic, "out_topic": out_topic}],
    )

    web = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        parameters=[{"port": int(port), "address": "0.0.0.0"}],
    )

    actions = [gscam, mux, web]

    if fps_metric in ("true", "1", "yes"):
        # Reutilise TEL QUEL le noeud de metrique du chantier 1, par CHEMIN ABSOLU : un
        # fichier neuf sous le bind-mount n'est pas lie dans install/share sans re-passage de
        # colcon. On le copie pas -- une troisieme copie du meme fichier serait une dette.
        # Il mesure la SORTIE du mux (donc ce que voit reellement le navigateur), et
        # echantillonne le flux HTTP par salves pour ne pas epingler web_video_server en
        # re-encodage continu.
        base_out = out_topic[:-len("/compressed")] \
            if out_topic.endswith("/compressed") else out_topic
        actions.append(ExecuteProcess(
            cmd=[
                "python3",
                "/root/linorobot2_ws/src/linorobot2/linorobot2_bringup/scripts/"
                "camera_fps_node.py",
                "--ros-args",
                "-p", "image_topic:=" + out_topic,
                "-p", "capture_type:=compressed",
                "-p", "stream_type:=ros_compressed",
                "-p", "stream_topic:=" + base_out,
                "-p", "stream_port:=" + port,
            ],
            output="screen",
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_YBStm32",
            description="Nom du fichier de source unique dans bamboo_base/config/robots/."),
        DeclareLaunchArgument(
            "stream_mode", default_value="mjpg",
            description="mjpg = passthrough MJPG dans ROS, tracking POSSIBLE ; "
                        "h264 = MediaMTX garde la camera, tracking IMPOSSIBLE."),
        DeclareLaunchArgument(
            "video_device", default_value="/dev/bamboocam",
            description="Peripherique V4L2 (regle udev sur 05a3:9331 ; /dev/video0 en repli)."),
        DeclareLaunchArgument(
            "framerate", default_value="30",
            description="Cadence demandee au capteur en MJPG (fps)."),
        DeclareLaunchArgument(
            "frame_id", default_value="camera_link",
            description="Repere TF porte par les trames."),
        DeclareLaunchArgument(
            "port", default_value="8080",
            description="Port HTTP du web_video_server. PARTAGE avec camera.real et "
                        "camera.mjpg, mutuellement exclusifs."),
        DeclareLaunchArgument(
            "raw_topic", default_value="/video/raw/compressed",
            description="Sortie de gscam2 et entree du mux. C'est aussi ce que le groupe "
                        "bamboo_videotracking souscrit."),
        DeclareLaunchArgument(
            "out_topic", default_value="/video/stream/compressed",
            description="L'URL STABLE : brut ou enrichi selon le registre du mux."),
        DeclareLaunchArgument(
            "enable_fps_metric", default_value="true",
            description="Publier /camera/capture_fps et /camera/stream_fps."),
        OpaqueFunction(function=_launchSetup),
    ])
