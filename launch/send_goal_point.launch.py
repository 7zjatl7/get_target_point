from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    _target_point = get_package_share_directory('get_target_point')
    
    _yaml_path = os.path.join(
        _target_point,
        'configs',
        'params.yaml'
    )

    # DeclareLaunchArgument의 이름이 'declare_yaml_path'인 것을 'yaml_path'로 변경
    _declare_yaml_path_argument = DeclareLaunchArgument(
        'yaml_path',  # 여기를 수정: 'declare_yaml_path' -> 'yaml_path'
        default_value=_yaml_path,
        description='Path to the configuration YAML file.'
    )

    # LaunchConfiguration의 이름도 'yaml_path'로 일치시켜야 함
    _yaml = LaunchConfiguration('yaml_path')

    return LaunchDescription([
        _declare_yaml_path_argument,  # LaunchDescription에 DeclareLaunchArgument를 추가해야 함
        Node(
            package='get_target_point',
            executable='target_point',
            name='get_skeletons_node',
        ),
    ])