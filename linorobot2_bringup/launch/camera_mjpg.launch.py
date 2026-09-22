# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Chemin camera MJPG->ROS a 30 fps (etape 2). Contrairement a camera.launch.py (v4l2_camera
# YUYV->rgb8 puis web_video_server re-encode -> plafond ~12 fps, 4 coeurs RPi4 satures), on
# publie ici le MJPG **materiel** de la webcam SANS le decoder :
#   gscam2 (pipeline GStreamer v4l2src ! image/jpeg, image_encoding=jpeg -> copie le buffer
#   JPEG dans un sensor_msgs/CompressedImage sur /image_raw/compressed, ZERO decodage)
#   -> web_video_server type=ros_compressed ressert ces frames JPEG telles quelles.
# Aucun decode ni ré-encode sur le RPi -> cadence camera (30 fps) preservee a coup CPU quasi nul.
#
# CONTRAINTE UVC : /dev/video0 n'accepte qu'UN format actif a la fois -> ce chemin est
# MUTUELLEMENT EXCLUSIF avec camera.real (v4l2) et camera.h264 (MediaMTX). L'orchestrateur en
# demarre un seul a la fois. gscam2 en mode jpeg ne publie QUE /image_raw/compressed (pas de
# /image_raw brut) -> le node FPS date la capture en CompressedImage (capture_type=compressed).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='video_device',
            default_value='/dev/video0',
            description='Peripherique V4L2 de la camera'
        ),
        DeclareLaunchArgument(
            name='width', default_value='640', description='Largeur MJPG'),
        DeclareLaunchArgument(
            name='height', default_value='480', description='Hauteur MJPG'),
        DeclareLaunchArgument(
            name='framerate', default_value='30', description='Cadence MJPG (fps)'),
        DeclareLaunchArgument(
            name='frame_id', default_value='camera_link', description='Repere TF de la camera'),
        DeclareLaunchArgument(
            name='port', default_value='8080', description='Port HTTP du web_video_server'),
        DeclareLaunchArgument(
            name='enable_fps_metric', default_value='true',
            description='Publier /camera/capture_fps et /camera/stream_fps (metrique FPS)'),

        # gscam2 en passthrough JPEG. gscam_config = la portion de pipeline AVANT l'appsink que
        # gscam2 ajoute lui-meme ; on force les caps image/jpeg pour que v4l2src negocie le MJPG
        # materiel (pas de conversion). image_encoding=jpeg -> gscam2 publie CompressedImage sur
        # image_raw/compressed en recopiant le buffer (cf. gscam_node.cpp : branche "jpeg").
        Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam_node',
            output='screen',
            parameters=[{
                'gscam_config': [
                    'v4l2src device=', LaunchConfiguration('video_device'),
                    ' do-timestamp=true ! image/jpeg,width=', LaunchConfiguration('width'),
                    ',height=', LaunchConfiguration('height'),
                    ',framerate=', LaunchConfiguration('framerate'), '/1',
                ],
                'image_encoding': 'jpeg',
                'camera_name': 'camera',
                'frame_id': LaunchConfiguration('frame_id'),
                'sync_sink': False,  # ne pas bloquer sur l'horloge : on veut le debit brut
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

        # Metrique FPS : source compressee -> capture datee en CompressedImage sur
        # /image_raw/compressed ; stream mesure en type=ros_compressed (frames resservies telles
        # quelles). Lance par chemin absolu (fichier neuf sans symlink), comme camera.launch.py.
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('enable_fps_metric')),
            cmd=[
                'python3',
                '/root/linorobot2_ws/src/linorobot2/linorobot2_bringup/scripts/camera_fps_node.py',
                '--ros-args',
                '-p', 'image_topic:=/image_raw/compressed',
                '-p', 'capture_type:=compressed',
                '-p', 'stream_type:=ros_compressed',
                '-p', ['stream_port:=', LaunchConfiguration('port')],
            ],
            output='screen',
        ),
    ])
