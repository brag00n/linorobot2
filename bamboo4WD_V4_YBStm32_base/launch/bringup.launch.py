"""Bringup du ROBOT bambooSTM32YB : tous les noeuds de cette machine, en un point d'entree.

CE QUE CE FICHIER EST, ET CE QU'IL N'EST PAS
--------------------------------------------
Il declare la COMPOSITION de cette machine : quelles cartes elle embarque, quels groupes
fonctionnels elle fait tourner. Il ne porte AUCUNE valeur physique -- ni geometrie, ni PID,
ni bornes de servo : tout cela vit dans bamboo_base/config/robots/<robot>.yaml, la source
unique, et est propage par le seul argument `robot`.

Il coexiste avec les services Docker de docker/docker-compose.yaml, et les deux ne font PAS
le meme travail -- a ne pas confondre, sous peine de lancer deux fois les memes noeuds :

  * en EXPLOITATION, chaque groupe tourne dans SON conteneur (`bamboo_video`,
    `bamboo_videotracking`, `driver.stm32`), avec sa propre politique de redemarrage et son
    propre `colcon build`. C'est ce decoupage qui permet d'arreter le tracking sans couper
    le flux video, exigence explicite de ce chantier.
  * ce launch-ci est l'entree MONO-PROCESSUS (banc, mise au point, `docker compose run`) :
    un seul `ros2 launch` leve la machine entiere, sans decouverte DDS inter-conteneurs.
    C'est aussi la DEFINITION lisible de "tous les noeuds du robot".

Les deux chemins incluent les MEMES launch de groupe : il n'y a pas deux configurations a
maintenir, seulement deux manieres de les demarrer.

DEFAUTS, ET POURQUOI ILS SONT PRUDENTS
--------------------------------------
  enable_driver       true   la carte en LECTURE SEULE (cf. enable_cmd_vel)
  enable_description  true   URDF + TF, sans effet materiel
  enable_video        true   acquisition + diffusion, autonome
  enable_tracking     FALSE  regle de securite : ce groupe est le SEUL a commander les
                             servos. Tant que T6/T7 de bambooSTM32YB ne sont pas passes,
                             une camera qui part en butee au demarrage est un risque reel
                             (SG90 bloque a ~700 mA, pignons plastique). A armer a la main.
  enable_control      FALSE  le paquet bamboo_control n'existe pas encore (lot V5). Mis a
                             true avant sa creation, le launch echoue en nommant le paquet
                             -- ce qui est le bon comportement, pas un bug a contourner.
  enable_cmd_vel      false  les MOTEURS restent muets : le pack 12,6 V arrive brut sur des
                             moteurs 7,4 V nominaux (constantes PWM non mises a l'echelle).
                             LES SERVOS OUI, LES MOTEURS NON, jusqu'a T2/T3/T5.

Exemples :
  ros2 launch bamboo4WD_V4_YBStm32_base bringup.launch.py
  ros2 launch bamboo4WD_V4_YBStm32_base bringup.launch.py enable_tracking:=true
  ros2 launch bamboo4WD_V4_YBStm32_base bringup.launch.py enable_video:=false   # carte seule
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from bamboo_base.capability_check import checkServoAxes


def _include(package, launch_file, arguments, flag):
    """Inclusion conditionnelle d'un launch de groupe.

    Le chemin est une SUBSTITUTION, donc resolu seulement si la condition est vraie : un
    groupe desactive dont le paquet n'est pas installe (bamboo_control avant le lot V5) ne
    fait donc rien echouer. C'est ce qui permet de declarer des maintenant la place du
    groupe manette sans attendre son paquet.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])),
        # .items() et pas le dict : IncludeLaunchDescription itere des paires (nom, valeur)
        # et un dict nu s'itere sur ses CLES -- l'erreur se voit seulement au lancement.
        launch_arguments=arguments.items(),
        condition=IfCondition(LaunchConfiguration(flag)),
    )


def _checkCapabilities(context, *args, **kwargs):
    """Confronte servo_axes aux capacites DECLAREES des modules de controleur vises.

    Se fait ICI, au tout debut du bringup, et pas dans un noeud : un axe pose sur une carte
    sans servo PWM ne doit pas etre decouvert quand la premiere consigne part -- la
    WaveShare ESP32, par exemple, repond UNSUPPORTED a DO_SET_SERVO en renvoyant quand
    meme un ACK, donc rien ne bouge et rien ne se plaint.

    Lecture de fichiers UNIQUEMENT : rien n'ouvre le port serie, qui est mono-proprietaire.
    Ce controle tourne meme quand le tracking est desactive, parce qu'un axe mal declare est
    une erreur de configuration, pas une consequence des drapeaux du lancement.
    """
    robot = LaunchConfiguration("robot").perform(context)
    return [LogInfo(msg="capacites : " + line) for line in checkServoAxes(robot)]


def generate_launch_description():
    robot = LaunchConfiguration("robot")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_YBStm32",
            description="Nom du fichier de source unique dans bamboo_base/config/robots/ "
                        "(sans .yaml). Propage depuis docker/.env (ROBOT=) et transmis TEL "
                        "QUEL a tous les groupes : c'est la seule bascule."),
        DeclareLaunchArgument(
            "enable_driver", default_value="true",
            description="Driver de la carte Yahboom v3 (MAVLink sysid 1). Proprietaire "
                        "UNIQUE du CH340 : arreter robot_controlv3 et le MCP robot-action "
                        "avant."),
        DeclareLaunchArgument(
            "enable_description", default_value="true",
            description="robot_state_publisher (URDF + TF). publish_joints=false : ce sont "
                        "les encodeurs, via le driver, qui publient les vrais joint_states."),
        DeclareLaunchArgument(
            "enable_video", default_value="true",
            description="Groupe video : acquisition MJPG + mux + diffusion HTTP."),
        DeclareLaunchArgument(
            "enable_tracking", default_value="false",
            description="Groupe tracking. FALSE par defaut, et c'est une regle de securite "
                        "(voir l'en-tete) : il commande les servos. A armer apres T6/T7."),
        DeclareLaunchArgument(
            "enable_control", default_value="false",
            description="Groupe manette. Le paquet bamboo_control arrive au lot V5 ; "
                        "true avant cela fait echouer le launch en le nommant."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="Actuation MOTEUR. Reste false jusqu'a T2/T3/T5 de bambooSTM32YB. "
                        "N'affecte PAS les servos."),
        DeclareLaunchArgument(
            "video_device", default_value="/dev/video0",
            description="Peripherique V4L2. /dev/bamboocam des que la regle udev "
                        "99-bambooSTM32YB.rules est posee sur la machine ; /dev/video0 en "
                        "repli, qui est l'etat constate du RPi a T10."),

        # --- controle de coherence AVANT tout noeud --------------------------------------
        OpaqueFunction(function=_checkCapabilities),

        # --- la carte -----------------------------------------------------------------
        _include("bamboo_controler_YBStm32v3", "driver.launch.py",
                 {"robot": robot,
                  "enable_cmd_vel": LaunchConfiguration("enable_cmd_vel")},
                 "enable_driver"),

        # --- URDF / TF ----------------------------------------------------------------
        # Amont linorobot2, parametre par ROBOT_BASE (4wd) et non par `robot` : c'est la
        # FAMILLE cinematique qu'il lit, pas l'identite de la machine.
        _include("linorobot2_description", "description.launch.py",
                 {"rviz": "false", "publish_joints": "false"},
                 "enable_description"),

        # --- groupe video -------------------------------------------------------------
        # stream_mode force a mjpg : en h264 le flux ne passe pas par ROS et le tracking
        # devient impossible (contrainte UVC, un seul format a la fois sur le capteur).
        _include("bamboo_video", "video.launch.py",
                 {"robot": robot,
                  "stream_mode": "mjpg",
                  "video_device": LaunchConfiguration("video_device")},
                 "enable_video"),

        # --- groupe tracking ----------------------------------------------------------
        # Aucun `depends_on` implicite : s'il demarre sans le groupe video, il attend son
        # flux sans rien casser, et le mux sert le brut tant qu'il ne s'est pas enregistre.
        _include("bamboo_videotracking", "videotracking.launch.py",
                 {"robot": robot},
                 "enable_tracking"),

        # --- groupe manette (lot V5) ---------------------------------------------------
        _include("bamboo_control", "control.launch.py",
                 {"robot": robot},
                 "enable_control"),
    ])
