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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Webcam USB UVC (05a3:9331) : formats MJPG/YUYV/H264. v4l2_camera 0.6.2 ne sait capturer
    # que des formats bruts (MJPG -> crash "not supported yet"), donc YUYV 640x480 converti en
    # rgb8 -> /image_raw. web_video_server (:8080) encode ensuite H.264/VP8/MJPEG a la volee
    # pour la diffusion distante. (Passer en MJPG/H264 natif demanderait usb_cam a la place.)
    return LaunchDescription([
        DeclareLaunchArgument(
            name='video_device',
            default_value='/dev/video0',
            description='Peripherique V4L2 de la camera'
        ),
        DeclareLaunchArgument(
            name='pixel_format',
            default_value='YUYV',
            description='Format de capture V4L2 (YUYV | MJPEG | H264)'
        ),
        DeclareLaunchArgument(
            name='frame_id',
            default_value='camera_link',
            description='Repere TF de la camera'
        ),
        DeclareLaunchArgument(
            name='port',
            default_value='8080',
            description='Port HTTP du web_video_server'
        ),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'image_size': [640, 480],
                'camera_frame_id': LaunchConfiguration('frame_id'),
                'output_encoding': 'rgb8',
            }]
        ),

        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'address': '0.0.0.0',
            }]
        ),
    ])
