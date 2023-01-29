import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('odrivelib'),
        'config',
        'odrive_init.yaml'
    )
    return LaunchDescription([
        Node(
            package='odrivelib',
            executable='node_odrive_srv',
            # name="node_odrive",
            output='screen',
            parameters = [config]
        ),
    ])