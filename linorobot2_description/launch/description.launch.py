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
    """
    robot = os.getenv('BAMBOO_ROBOT', 'bamboo4WD_V4_WSEsp32')
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('bamboo_base'),
                            'config', 'robots', f'{robot}.yaml')
        with open(path) as fh:
            params = yaml.safe_load(fh)['/**']['ros__parameters']
        return float(params['wheel_diameter_m']) / 2.0, float(params['wheel_separation_m']) / 2.0
    except Exception:
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