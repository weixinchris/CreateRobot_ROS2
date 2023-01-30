import rclpy
import sys
from rclpy.node import Node
from odrive.enums import *
from std_srvs.srv import Trigger
from odrivelib import constants
from odrive_interfaces.srv import AxisState, PositionControl, VelocityControl

class odriveTest(Node):
    def __init__(self,name):
        super().__init__(name)
        # print(*constants.ODRIVE_SERVICE_API_LIST,sep='\n')
        
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

        self.state_request = AxisState.Request()  
        self.state_response = AxisState.Response()
        self.connect_response = Trigger.Response()
        self.connect_request = Trigger.Request()

    def send_connect_request(self):
        self.connect_future = self.connect_client.call_async(self.connect_request)
        rclpy.spin_until_future_complete(self, self.connect_future)
        return self.connect_future.result()

    def send_requestState_request(self):
        axis_num = int(input("Please enter the axis number (0 or 1): "))
        for state_index in constants.REQUEST_STATE_LIST:
            print(state_index)
        odrivestate = int(input("Please enter the state (0-13): "))
        self.state_request.axis = axis_num
        self.state_request.state = odrivestate
        
        self.state_future = self.state_client.call_async(self.state_request)
        rclpy.spin_until_future_complete(self,self.state_future)
        return self.state_future.result()
    
    def response_connect(self):
        response = self.send_connect_request()
        self.get_logger().info('Odrive initialization result: %d' %(response.success))
        if(response.success == True):
            self.get_logger().info("Connection successful!")
            self.get_logger().info('Odrive Serial Number: %s' %(response.message))
    
    def response_requestState(self):
        response = self.send_requestState_request()
        self.get_logger().info('Request State: %d' %(response.success))
        if(response.success == True):
            self.get_logger().info("Request state successful!")
            self.get_logger().info(f'Current state: {response.message}')
    
def main(args=None):
    rclpy.init(args=args)
    node = odriveTest("node_odrive_cli")
    while rclpy.ok(): 
        for api_index in constants.ODRIVE_SERVICE_API_LIST:
            print(f'{api_index}')
        api_num = input("Enter API Index: ")
        match int(api_num):
            case 1:
                node.response_connect() 
            case 2:
                node.response_requestState()
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()