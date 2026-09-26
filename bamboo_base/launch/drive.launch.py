"""Traction a la manette : driver + joy_linux_node + teleop_twist_joy, EN UN SEUL PROCESSUS DE LAUNCH.

UN SEUL CONTENEUR, ET C'EST LE MOTIF DU FICHIER (lot 5.2 du chantier 1). Le reseau compose est
en `bridge` et la memoire partagee DDS ne traverse pas la frontiere du conteneur : un
`joy_linux_node` dans un conteneur et le driver dans un autre mettraient la DECOUVERTE DDS sur
le chemin critique du pilotage -- exactement la chose qu'on ne veut pas entre un homme-mort et
des moteurs. Les trois noeuds vivent donc ensemble.

CE FICHIER NE DUPLIQUE PAS LE DRIVER : il INCLUT `driver.launch.py`, qui porte seul le contrat
d'ordre des fichiers de parametres (fichier d'hote puis canonique du robot, le second gagnant).
Recopier le `Node(...)` ici aurait cree une seconde verite pour la geometrie.

⚠️ `launch_arguments` recoit une LISTE DE PAIRES, jamais un dict : `IncludeLaunchDescription`
itere l'objet, et un dict s'itere sur ses CLES -- le defaut ne se voit qu'au lancement (defaut
reellement rencontre dans le bringup de bambooSTM32YB).

L'ARMEMENT RESTE SEPARE DU DEADMAN, deliberement. `enable_cmd_vel` garde son defaut **false** :
lancer le pilotage ne doit pas armer les moteurs. Le deadman est un garde-fou d'OPERATEUR (il
borne ce qui est publie) ; `enable_cmd_vel` est un garde-fou de MACHINE (il decide si la
souscription /cmd_vel existe cote driver). Sur BambooWS l'attache moteur de M1 est cassee :
`drive.launch.py` peut se lancer et publier, aucune roue ne doit bouger tant que l'armement
n'est pas demande explicitement -- et il est commutable a chaud par parametre, donc rien
n'oblige a le poser ici.

Perimetre : robot **BambooWS** (`bamboo4WD_V4_WSEsp32`). Le robot bambooSTM32YB a sa propre
manette (`bamboo_control`), qui ne publie AUCUN /cmd_vel et ne remplace pas ce fichier.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("bamboo_base")
    # `joy_drive.yaml` et NON `joy_bamboo.yaml` : ce dernier nom est pris par bamboo_control
    # (manette du tracking, aucun /cmd_vel). Deux homonymes s'installeraient sans conflit
    # technique et se corrigeraient l'un pour l'autre a la premiere recherche.
    joy_cfg = os.path.join(share, "config", "joy_drive.yaml")
    driver_launch = os.path.join(share, "launch", "driver.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="bamboo4WD_V4_WSEsp32",
            description="Nom du fichier canonique dans config/robots/ (sans .yaml). Meme "
                        "defaut que driver.launch.py : ce launch est celui de BambooWS."),
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="Armement MOTEUR, independant du deadman. false = le driver ne cree "
                        "meme pas la souscription /cmd_vel -- la manette publie dans le vide, "
                        "ce qui est le bon etat pour verifier le mapping sans rien faire "
                        "bouger. A armer seulement roues surelevees, ou au sol apres "
                        "validation du deadman."),
        DeclareLaunchArgument(
            "joy_dev", default_value="/dev/input/js0",
            description="Peripherique manette. Fait de MACHINE, visible dans le conteneur par "
                        "`privileged` + /dev:/dev ; le numero depend de l'ordre d'appairage, a "
                        "relever a T9 de BambooWS."),

        LogInfo(msg=["traction manette : deadman OBLIGATOIRE (LB tenu), armement moteur "
                     "enable_cmd_vel=", LaunchConfiguration("enable_cmd_vel"),
                     ". Un deadman relache publie un Twist NUL ; une manette perdue laisse "
                     "agir le watchdog 0,5 s du driver."]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments=[
                ("robot", LaunchConfiguration("robot")),
                ("enable_cmd_vel", LaunchConfiguration("enable_cmd_vel")),
            ],
        ),

        # joy_linux et NON joy : meme type de message, index d'axes DIFFERENTS. Les deux ne
        # sont pas substituables sans re-relever la table au jstest.
        Node(
            package="joy_linux", executable="joy_linux_node", name="joy_linux_node",
            parameters=[joy_cfg, {"dev": LaunchConfiguration("joy_dev")}],
            output="screen",
        ),
        # Le nom du noeud doit rester `teleop_twist_joy` : c'est la CLE du bloc de parametres
        # dans joy_drive.yaml. Le renommer rendrait le fichier inoperant en silence -- le
        # noeud demarrerait avec ses defauts, dont require_enable_button... a false.
        Node(
            package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy",
            parameters=[joy_cfg],
            output="screen",
        ),
    ])
