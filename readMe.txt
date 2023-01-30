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

Add a client

self.connect_client = self.create_client(
    Trigger,
    'connect_odrive'
)
self.state_client = self.create_client(
    AxisState,
    'request_state'
)
while not self.connect_client.wait_for_service(timeout_sec=1.0):                  
    self.get_logger().info('Odrive connect service is not available.')
while not self.state_client.wait_for_service(timeout_sec=1.0):             
    self.get_logger().info('Odrive request_state is not available.')

NOTE: the name of the client has to be the same!
for example: the first client has a name of connect_client, the second client has a name of state_client