Load parameters from yaml file

1. In the node file, declare the parameters. Don't forget to declare its data type
self.declare_parameters(
            namespace='',
            parameters=[
                ('cur_limit',rclpy.Parameter.Type.DOUBLE),
                ('vel_limit',rclpy.Parameter.Type.DOUBLE),
                ('calibration_current',rclpy.Parameter.Type.DOUBLE)
            ]
        )

2. Create a yaml file in the root directory, usually called "config"
node_odrive:   ----->node name, which has to be the same as the node, here we have "node = OdriveNode("node_odrive")"
  ros__parameters:
    cur_limit: 16.0
    vel_limit: 6.0
    calibration_current: 3.0

3. 'console_scripts': [
            'odrive_node = odrivelib.node_odrive:main',
        ],
About the entry point, the first segment, "odrive_node" can be an arbitrary name, which has to be the same as the executable name in the LaunchDescription
return LaunchDescription([
        Node(
            executable='odrive_node',
        ),
    ])
The second segment is the package name "odrivelib", the third segment is the node name "node_odrive"

4. Add the directory of the yaml file in the launch file
from ament_index_python.packages import get_package_share_directory
config = os.path.join(
        get_package_share_directory('odrivelib'),
        'config',
        'odrive_init.yaml'
    )

5. Launch a node with yaml file loaded
ros2 run odrivelib odrive_node --ros-args --params-file ~/cr_ws/src/odrivelib/config/odrive_init.yaml