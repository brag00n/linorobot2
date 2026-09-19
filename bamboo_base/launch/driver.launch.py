"""Lance le driver ESP32 MAVLink seul (test L5 lecture seule / L6 actuation).

Par defaut enable_cmd_vel reste sur la valeur du YAML (false = lecture seule). Pour l'actuation
au L6 : passer l'argument de lancement enable_cmd_vel:=true (roues surelevees obligatoires).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("bamboo_base"), "config", "esp32_driver.yaml")

    enable_cmd_vel = LaunchConfiguration("enable_cmd_vel")

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_cmd_vel", default_value="false",
            description="true = actuation (L6, roues surelevees) ; false = lecture seule (L5)."),
        Node(
            package="bamboo_base",
            executable="esp32_mavlink_driver",
            name="esp32_mavlink_driver",
            output="screen",
            parameters=[cfg, {"enable_cmd_vel": enable_cmd_vel}],
        ),
    ])
