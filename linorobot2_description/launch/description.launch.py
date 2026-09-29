# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Publication de /robot_description et des TF, pour LE robot qu'on lui nomme.

QUI CHOISIT LE ROBOT (corrige le 2026-09-28, etape E1)
------------------------------------------------------
L'identite du robot arrive par l'argument de lancement `robot`, comme pour tous les autres
groupes. Avant, elle etait lue dans la variable d'environnement BAMBOO_ROBOT avec un repli
ECRIT EN DUR sur bamboo4WD_V4_WSEsp32 -- or un grep du depot entier montre que cette
variable n'est DEFINIE NULLE PART, seulement lue ici. Consequence mesuree : sur le STM32
comme sur la WaveShare, la TF portait la geometrie de roue de la WaveShare (rayon 0.04 m et
demi-voie 0.0625 m, au lieu de 0.03425 et 0.0825 pour le STM32).

La variable d'environnement reste un repli, pour ne casser aucun appel existant, mais elle
n'a plus de nom de robot en dur : sans argument ni variable, on retombe sur les defauts du
xacro, en le DISANT sur stderr.
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _canonicalParams(robot):
    """Lit la source unique <robot>.yaml et renvoie son bloc ros__parameters, ou None.

    Import et recherche de paquet sont volontairement tolerants : ce paquet ne DEPEND pas de
    bamboo_base, il l'utilise s'il est la -- cas d'une installation de la seule description,
    ou d'un robot amont linorobot2.

    DEUX chemins de recherche, et le second n'est pas du zele : constate le 2026-09-23, le
    conteneur robotdesc ne construit PAS bamboo_base (seul driver.real le fait, dans sa
    propre couche), donc l'index ament n'y connait pas ce paquet. On cherche donc aussi le
    paquet frere dans l'arbre SOURCE, atteint depuis ce fichier (l'install etant en
    --symlink-install, __file__ resout dans src/).
    """
    if not robot:
        return None
    rel = os.path.join('config', 'robots', robot + '.yaml')
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory('bamboo_base'), rel))
    except Exception:
        pass
    # <depot>/linorobot2_description/launch/ -> <depot>/bamboo_base/
    here = os.path.dirname(os.path.realpath(__file__))
    candidates.append(os.path.join(here, '..', '..', 'bamboo_base', rel))

    for path in candidates:
        try:
            import yaml
            with open(path) as fh:
                return yaml.safe_load(fh)['/**']['ros__parameters']
        except Exception:
            continue
    # Repli BRUYANT : un URDF sur valeurs constructeur fausse la TF, les costmaps Nav2 et le
    # plugin skid-steer sans rien casser de visible -- il faut donc le dire.
    print('[description.launch] ATTENTION : geometrie canonique introuvable '
          '(' + robot + '.yaml) -> le xacro utilise ses defauts constructeur.',
          file=sys.stderr)
    return None


def _describe(context, *args, **kwargs):
    """Resout le robot, puis la famille cinematique, puis l'URDF -- dans cet ordre.

    Tout se fait ICI et pas au niveau module : `robot` est desormais un argument de
    lancement, donc sa valeur n'existe qu'une fois le contexte disponible.
    """
    robot = LaunchConfiguration('robot').perform(context)
    if not robot:
        # Le repli DOIT etre bruyant, y compris ici : constate a TC1 le 2026-09-28, un
        # conteneur relance par `docker compose restart` garde la commande figee a sa
        # CREATION, donc sans robot:= -- on retombait sur les defauts du xacro (demi-voie
        # 0.086) sans un mot, ce qui est exactement le defaut qu'on corrige.
        print('[description.launch] ATTENTION : aucun robot nomme (ni robot:= ni '
              'BAMBOO_ROBOT) -> le xacro utilise ses defauts constructeur, la TF ne '
              'decrit AUCUN robot reel.', file=sys.stderr)
    params = _canonicalParams(robot) or {}

    # La FAMILLE cinematique vient du fichier du robot, pas de l'environnement : c'est une
    # propriete du ROBOT, et la garder dans LINOROBOT2_BASE en faisait une deuxieme source de
    # verite, qu'un .env d'hote pouvait contredire en silence. La variable reste le repli des
    # robots amont linorobot2, qui n'ont pas de fichier canonique.
    robot_base = params.get('robot_base') or os.getenv('LINOROBOT2_BASE') or ''
    if not robot_base:
        raise RuntimeError(
            'famille cinematique inconnue : ni robot_base: dans le fichier canonique ('
            + (robot or 'aucun robot nomme') + '), ni LINOROBOT2_BASE dans '
            "l'environnement. Passer robot:=<nom> au lancement.")

    # Geometrie passee en arguments xacro : l'URDF (donc la TF, les costmaps Nav2 et le
    # plugin skid-steer) tire du MEME fichier que le driver. Ne PAS editer
    # 4wd_properties.urdf.xacro a la main.
    # Plus de garde `if robot_base == '4wd'` : elle abandonnait les arguments EN SILENCE pour
    # toute autre famille. Un xacro qui ne declare pas ces arguments le dit lui-meme.
    xacro_args = []
    if 'wheel_diameter_m' in params and 'wheel_separation_m' in params:
        xacro_args = [' wheel_radius:=', str(float(params['wheel_diameter_m']) / 2.0),
                      ' wheel_pos_y:=', str(float(params['wheel_separation_m']) / 2.0)]

    # L'argument `urdf` garde le dernier mot ; vide (le defaut), il est deduit de la famille.
    urdf = LaunchConfiguration('urdf').perform(context) or PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'),
         'urdf/robots', robot_base + '.urdf.xacro'])

    return [
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints"))
            # parameters=[
            #     {'use_sim_time': LaunchConfiguration('use_sim_time')}
            # ] #since galactic use_sim_time gets passed somewhere and rejects this when defined from launch file
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', urdf] + xacro_args)
                }
            ]
        ),
    ]


def generate_launch_description():
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'rviz', 'description.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot',
            default_value=EnvironmentVariable('BAMBOO_ROBOT', default_value=''),
            description='Nom du fichier de source unique dans bamboo_base/config/robots/ '
                        '(sans .yaml). Donne la famille cinematique ET la geometrie de roue. '
                        'Vide : defauts du xacro, signale sur stderr.'
        ),

        DeclareLaunchArgument(
            name='urdf',
            default_value='',
            description='URDF path (vide = deduit de la famille cinematique du robot)'
        ),

        DeclareLaunchArgument(
            name='publish_joints',
            default_value='true',
            description='Launch joint_states_publisher'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        OpaqueFunction(function=_describe),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])

#sources:
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940
