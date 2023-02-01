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
        self.pos_cmd_client = self.create_client(
            PositionControl,
            'position_cmd'
        )
        while not self.connect_client.wait_for_service(timeout_sec=1.0):                  
            self.get_logger().info('Odrive connect service is not available.')
        while not self.state_client.wait_for_service(timeout_sec=1.0):             
            self.get_logger().info('Odrive request_state service is not available.')
        while not self.control_modes_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Odrive control_modes service is not available.')
        while not self.pos_cmd_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Odrive pos_cmd service is not available.')

        self.state_request = AxisState.Request()  
        self.state_response = AxisState.Response()
        self.connect_response = Trigger.Response()
        self.connect_request = Trigger.Request()
        self.control_modes_request = AxisModes.Request()
        self.control_modes_response = AxisModes.Response()
        self.pos_cmd_request = PositionControl.Request()
        self.pos_cmd_response = PositionControl.Response()

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
        rclpy.spin_until_future_complete(self, self.state_future)
        return self.state_future.result()
    
    def __send_control_modes_request(self):
        axis_num = int(input("Please enter the axis number (0 or 1): "))
        for control_mode_index in constants.INPUT_MODES_LIST:
            print(control_mode_index)
        control_mode = int(input("Please enter the control mode (0-8): "))
        self.control_modes_request.axis = axis_num
        self.control_modes_request.control_mode = control_mode
        match control_mode:
            case 2: 
                self.control_modes_request.vel_ramp_rate = 0.5
            case 3:
                self.control_modes_request.input_filter_bandwidth = 2.0
            case 5:
                self.control_modes_request.trap_traj_vel_limit = 2.0
                self.control_modes_request.trap_traj_accel_limit = 1.2
                self.control_modes_request.trap_traj_decel_limit = 1.2
                self.control_modes_request.trap_traj_inertia = 0.0

        self.control_modes_future = self.control_modes_client.call_async(self.control_modes_request)
        rclpy.spin_until_future_complete(self, self.control_modes_future)
        return self.control_modes_future.result()
    
    def __send_pos_cmd_request(self):
        axis_num = int(input("Please enter the axis number (0 or 1): "))
        input_turn = float(input("Please enter the number of turns: "))
        self.pos_cmd_request.axis = axis_num
        self.pos_cmd_request.turns = input_turn
        self.pos_cmd_future = self.pos_cmd_client.call_async(self.pos_cmd_request)
        rclpy.spin_until_future_complete(self, self.pos_cmd_future)
        return self.pos_cmd_future.result()
    
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

    def response_pos_cmd(self):
        response = self.__send_pos_cmd_request()
        if(response.success == True):
            self.get_logger().info("Position cmd successful!")
            self.get_logger().info(f'Move steps: {response.message}')
    
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
            case 4:
                node.response_pos_cmd()
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()