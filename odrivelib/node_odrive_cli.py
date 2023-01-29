import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
# from odrive_interfaces import AxisState

class odriveTest(Node):
    def __init__(self,name):
        super().__init__(name)
        self.client = self.create_client(
            Trigger,
            'connect_odrive'
        )

        # self.State_client = self.create_client(
        #     AxisState,
        #     'request_state'
        # )

        self.response = Trigger.Response()
        while not self.client.wait_for_service(timeout_sec=1):
            self.get_logger().info('Odrive service is not available.')
        self.request = Trigger.Request()

        # self.stateResponse = AxisState.Response()
        # self.stateRequest = AxisState.Request()
        # while not self.client.wait_for_service(timeout_sec=1):
        #     self.get_logger().info('Odrive Request Service is not available.')

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main(args=None):
    rclpy.init(args=args)
    node = odriveTest("node_odrive_cli")
    
    node.get_logger().info("Waiting for a service")
    response = node.send_request()
    node.get_logger().info(
        'Odrive initialization result: %d' %(response.success)
    )
    if(response.success == True):
        node.get_logger().info("Connection successful!")
        node.get_logger().info('Odrive Serial Number: %s' %(response.message))
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()