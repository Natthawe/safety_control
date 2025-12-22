import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'safety_control'
    
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'auto_stop_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='auto_stop_test',
            name='auto_stop_test',
            output='screen',
            parameters=[config_file]
        )
    ])