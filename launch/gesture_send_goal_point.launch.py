from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    _target_point = get_package_share_directory('get_target_point')
    

    return LaunchDescription([
        Node(
            package='get_target_point',
            executable='async_target_point',
            name='get_skeletons_node',
        ),
        Node(
            package='get_target_point',
            executable='action_recognition',
            name='action_recognition_node',
            output='screen',
            parameters=[{
                'MODEL_PATH': 'assets/checkpoint/epoch_3.pth',
                'HOST' : 'localhost',
                'PORT' : 12121,
            }]
        ),
    ])