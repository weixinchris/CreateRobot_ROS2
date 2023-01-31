import rclpy
import sys
from rclpy.node import Node
from odrive.enums import *
from std_srvs.srv import Trigger
from odrivelib import constants
from odrive_interfaces.srv import AxisState, AxisModes, PositionControl, VelocityControl

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
        self.control_modes_client = self.create_client(
            AxisModes,
            'control_modes'
        )
        while not self.connect_client.wait_for_service(timeout_sec=1.0):                  
            self.get_logger().info('Odrive connect service is not available.')
        while not self.state_client.wait_for_service(timeout_sec=1.0):             
            self.get_logger().info('Odrive request_state service is not available.')
        while not self.control_modes_client.wait_for_service(timeout_sec=1.1):
            self.get_logger().info('Odrive control_modes service is not available.')

        self.state_request = AxisState.Request()  
        self.state_response = AxisState.Response()
        self.connect_response = Trigger.Response()
        self.connect_request = Trigger.Request()
        self.control_modes_request = AxisModes.Request()
        self.control_modes_response = AxisModes.Response()

    def __send_connect_request(self):
        self.connect_future = self.connect_client.call_async(self.connect_request)
        rclpy.spin_until_future_complete(self, self.connect_future)
        return self.connect_future.result()

    def __send_requestState_request(self):
        axis_num = int(input("Please enter the axis number (0 or 1): "))
        for state_index in constants.REQUEST_STATE_LIST:
            print(state_index)
        odrivestate = int(input("Please enter the state (0-13): "))
        self.state_request.axis = axis_num
        self.state_request.state = odrivestate
        
        self.state_future = self.state_client.call_async(self.state_request)
        rclpy.spin_until_future_complete(self,self.state_future)
        return self.state_future.result()
    
    def __send_control_modes_request(self):
        axis_num = int(input("Please enter the axis number (0 or 1): "))
        for control_mode_index in constants.CONTROL_MODES_LIST:
            print(control_mode_index)
        control_mode = int(input("Please enter the control mode (0-8): "))
        self.control_modes_request.axis = axis_num
        self.control_modes_request.control_mode = control_mode

        self.control_modes_future = self.control_modes_client.call_async(self.control_modes_request)
        rclpy.spin_until_future_complete(self, self.control_modes_future)
        return self.control_modes_future.result()
    
    def response_connect(self):
        response = self.__send_connect_request()
        if(response.success == True):
            self.get_logger().info("Connection successful!")
            self.get_logger().info('Odrive Serial Number: %s' %(response.message))
    
    def response_requestState(self):
        response = self.__send_requestState_request()
        if(response.success == True):
            self.get_logger().info("Request state successful!")
            self.get_logger().info(f'Current state: {response.message}')

    def response_controlmodes(self):
        response = self.__send_control_modes_request()
        if(response.success == True):
            self.get_logger().info("Select a control mode successful!")
            self.get_logger().info(f'Control Mode: {response.message}')
    
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
            case 3:
                node.response_controlmodes()
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()