import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yellow_ball_follower',
            executable='ball_tracker_node',
            name='ball_tracker_node',
        ),
    ])