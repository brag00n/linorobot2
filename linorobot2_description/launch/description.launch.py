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

import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _canonical_geometry():
    """Lit la geometrie dans la source unique bamboo_base/config/robots/<robot>.yaml.

    Renvoie (wheel_radius, wheel_pos_y) en metres, ou (None, None) si la source est
    introuvable -- cas d'une installation de la seule description, ou d'un robot amont
    linorobot2 : le xacro retombe alors sur ses propres defauts. Import et recherche de
    paquet sont volontairement tolerants : ce paquet ne DEPEND pas de bamboo_base, il
    l'utilise s'il est la.

    DEUX chemins de recherche, et le second n'est pas du zele : constate le 2026-09-23,
    le conteneur `robotdesc` ne construit PAS bamboo_base (seul `driver.real` le fait, dans
    sa propre couche), donc l'index ament n'y connait pas ce paquet. Le xacro retombait
    alors sur ses defauts constructeur -- identiques par chance aux valeurs canoniques,
    donc le repli etait INVISIBLE. On cherche donc aussi le paquet frere dans l'arbre
    SOURCE, atteint depuis ce fichier (l'install etant en --symlink-install, __file__
    resout dans src/).
    """
    robot = os.getenv('BAMBOO_ROBOT', 'bamboo4WD_V4_WSEsp32')
    rel = os.path.join('config', 'robots', f'{robot}.yaml')
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
                params = yaml.safe_load(fh)['/**']['ros__parameters']
            return (float(params['wheel_diameter_m']) / 2.0,
                    float(params['wheel_separation_m']) / 2.0)
        except Exception:
            continue
    # Repli BRUYANT : un URDF sur valeurs constructeur fausse la TF, les costmaps Nav2 et
    # le plugin skid-steer sans rien casser de visible -- il faut donc le dire.
    print('[description.launch] ATTENTION : geometrie canonique introuvable '
          f'({robot}.yaml) -> le xacro utilise ses defauts constructeur.', file=sys.stderr)
    return None, None


def generate_launch_description():
    robot_base = os.getenv('LINOROBOT2_BASE')

    # Geometrie passee en arguments xacro : l'URDF (donc la TF, les costmaps Nav2 et le
    # plugin skid-steer) tire du MEME fichier que le driver. Ne PAS editer
    # 4wd_properties.urdf.xacro a la main.
    wheel_radius, wheel_pos_y = _canonical_geometry()
    xacro_args = []
    if robot_base == '4wd' and wheel_radius is not None:
        xacro_args = [' wheel_radius:=', str(wheel_radius),
                      ' wheel_pos_y:=', str(wheel_pos_y)]

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "urdf/robots", f"{robot_base}.urdf.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'rviz', 'description.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
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
                    'robot_description': Command(
                        ['xacro ', LaunchConfiguration('urdf')] + xacro_args)
                }
            ]
        ),

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