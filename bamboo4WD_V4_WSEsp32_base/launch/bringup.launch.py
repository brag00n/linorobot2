"""Bringup du ROBOT BambooWS : tous les noeuds de cette machine, en un point d'entree.

/!\ ROBOT EN PAUSE ET BRINGUP JAMAIS LANCE. L'attache moteur N20 de M1 est cassee, ce qui
fausse toute odometrie au sol, et la consigne est explicite : AUCUN ESSAI AU SOL avant
reparation. Ce fichier existe pour que les deux robots aient la MEME forme -- c'est ce qui
rend la bascule `ROBOT=` credible -- pas pour etre demarre aujourd'hui. Les valeurs de ce
launch sont donc des choix de STRUCTURE, a confronter a la machine quand elle reviendra.

Pendant de bamboo4WD_V4_YBStm32_base/launch/bringup.launch.py, dont il partage la doctrine :
aucune valeur physique ici (tout est dans bamboo_base/config/robots/<robot>.yaml, propage
par le seul argument `robot`), et une inclusion conditionnelle par groupe.

CE QUI DIFFERE DE bambooSTM32YB, ET POURQUOI -- c'est la seule chose interessante dans ce
fichier, le reste est du calque :
  * PAS de groupe tracking. Cette carte n'a AUCUN servo PWM : DO_SET_SERVO repond
    MAV_RESULT_UNSUPPORTED (ConnectorMavlink.cpp:450). Il n'y a donc pas de camera
    motorisee sur ce robot, et l'inclure serait affirmer le contraire. La camera motorisee
    appartient a bambooSTM32YB -- regle de perimetre documentaire, pas un detail.
  * groupe video a FALSE par defaut. La chaine video de ce robot existe deja, mais sous
    forme de SERVICES compose eprouves (`camera.real`, `camera.mjpg`, `camera.h264` avec
    MediaMTX pour le H.264 a 30 fps). `bamboo_video` les remplacerait sans rien apporter
    ici, et son fichier canonique n'a d'ailleurs pas encore de cles `camera_*` : l'activer
    echouerait en nommant le parametre manquant, ce qui est le bon comportement mais pas
    une raison de l'activer.
  * lidar, SLAM et navigation restent DEHORS, eux aussi en services compose
    (`slam.real`, `navigation.real`). Les mettre ici demanderait de reprendre leur
    parametrage, qui n'est pas du ressort de ce lot.

DEFAUTS
  enable_driver       true   carte en LECTURE SEULE (cf. enable_cmd_vel)
  enable_description  true   URDF + TF, sans effet materiel
  enable_video        FALSE  voir ci-dessus : les services compose s'en chargent
  enable_control      FALSE  bamboo_control existe (lot V5) mais son mapping vise la
                             CAMERA MOTORISEE, que ce robot n'a pas (aucun servo PWM)
  enable_cmd_vel      FALSE  et ici ce n'est pas une precaution de principe : M1 est
                             CASSE. Aucune actuation avant remontage.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from bamboo_base.capability_check import checkServoAxes


def _include(package, launch_file, arguments, flag):
    """Inclusion conditionnelle d'un launch de groupe (meme helper que l'autre bringup).

    Le chemin est une SUBSTITUTION : un groupe desactive dont le paquet n'est pas installe
    ne fait rien echouer. `.items()` et pas le dict -- IncludeLaunchDescription itere des
    paires (nom, valeur) et un dict nu s'itere sur ses CLES.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])),
        launch_arguments=arguments.items(),
        condition=IfCondition(LaunchConfiguration(flag)),
    )


def _checkCapabilities(context, *args, **kwargs):
    """Meme controle que sur l'autre robot, et il a ici une valeur de NON-REGRESSION.

    Le fichier canonique de ce robot n'a pas de table `servo_axes`, ce qui est exact : la
    carte n'a pas de servo. Le controle passe donc en disant qu'il n'y a rien a verifier.
    Le jour ou quelqu'un ajouterait un axe ici, il echouerait EN NOMMANT L'AXE au lieu
    d'envoyer des consignes que la carte refuse tout en acquittant.
    """
    robot = LaunchConfiguration("robot").perform(context)
    return [LogInfo(msg="capacites : " + line) for line in checkServoAxes(robot)]


def generate_launch_description():
    robot = LaunchConfiguration("robot")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_WSEsp32",
            description="Nom du fichier de source unique dans bamboo_base/config/robots/ "
                        "(sans .yaml). counts_per_rev attendu : 2100, et non 1320."),
        DeclareLaunchArgument(
            "enable_driver", default_value="true",
            description="Driver ESP32 (MAVLink sysid 2), via l'enveloppe "
                        "bamboo_controler_WSEsp32. Proprietaire UNIQUE du CP2102."),
        DeclareLaunchArgument(
            "enable_description", default_value="true",
            description="robot_state_publisher (URDF + TF)."),
        DeclareLaunchArgument(
            "enable_video", default_value="false",
            description="Groupe video generique. FALSE : ce robot a deja ses services "
                        "compose camera.real / camera.mjpg / camera.h264."),
        DeclareLaunchArgument(
            "enable_control", default_value="false",
            description="Groupe manette. bamboo_control existe, mais son mapping vise "
                        "les axes servo de la camera motorisee : cette carte n'a "
                        "aucun servo PWM, donc les nudges seraient refuses."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="Actuation MOTEUR. Reste false : l'attache moteur de M1 est "
                        "CASSEE, aucun essai au sol avant reparation."),
        DeclareLaunchArgument(
            "video_device", default_value="/dev/video0",
            description="Peripherique V4L2, sans objet tant que enable_video est false."),

        # --- controle de coherence AVANT tout noeud --------------------------------------
        OpaqueFunction(function=_checkCapabilities),

        # --- la carte -----------------------------------------------------------------
        _include("bamboo_controler_WSEsp32", "driver.launch.py",
                 {"robot": robot,
                  "enable_cmd_vel": LaunchConfiguration("enable_cmd_vel")},
                 "enable_driver"),

        # --- URDF / TF ----------------------------------------------------------------
        _include("linorobot2_description", "description.launch.py",
                 {"rviz": "false", "publish_joints": "false"},
                 "enable_description"),

        # --- groupe video (desactive par defaut) ---------------------------------------
        _include("bamboo_video", "video.launch.py",
                 {"robot": robot,
                  "stream_mode": "mjpg",
                  "video_device": LaunchConfiguration("video_device")},
                 "enable_video"),

        # --- groupe manette (lot V5) ---------------------------------------------------
        _include("bamboo_control", "control.launch.py",
                 {"robot": robot},
                 "enable_control"),
    ])
