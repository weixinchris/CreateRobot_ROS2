from __future__ import print_function
import sys
from sensor_msgs.msg import JointState 
import rclpy
import time
import odrive
from odrive.enums import *
from odrivelib import constants
from rclpy.node import Node
from std_srvs.srv import Trigger
from odrive_interfaces.srv import AxisState, AxisModes, PositionControl, VelocityControl

class OdriveNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.__is_control_mode_service_called = False
        self.declare_parameters(
            namespace='',
            parameters=[
                ('cur_limit',rclpy.Parameter.Type.DOUBLE),
                ('vel_limit',rclpy.Parameter.Type.DOUBLE),
                ('calibration_current',rclpy.Parameter.Type.DOUBLE),
                ('brake_resistor',rclpy.Parameter.Type.INTEGER),
                ('dc_max_negative_current',rclpy.Parameter.Type.DOUBLE),
                ('pole_pairs',rclpy.Parameter.Type.INTEGER),
                ('torque_constant',rclpy.Parameter.Type.DOUBLE),
                ('cpr',rclpy.Parameter.Type.INTEGER),
                ('connection_timeout',rclpy.Parameter.Type.INTEGER),
                ('joint_state.topic',rclpy.Parameter.Type.STRING)
            ]
        )

        self.Node_odrive = odrive.find_any(
                timeout=self.get_parameter(
                    'connection_timeout'
                ).get_parameter_value().integer_value
            )

        self.get_logger().info("odrive service - connect")
        self.connect_odrive_service = self.create_service(
            Trigger,
            'connect_odrive',
            self.__connect_odrive_callback
        ) 

        self.get_logger().info("odrive service - request_state")
        self.odrive_request_state_service = self.create_service(
            AxisState,
            'request_state',
            self.__request_state_callback
        )

        self.get_logger().info("odrive service - control modes")
        self.odrive_control_modes_service = self.create_service(
            AxisModes,
            'control_modes',
            self.__control_modes_callback
        )

        self.get_logger().info("odrive service - position control")
        self.position_cmd_service = self.create_service(
            PositionControl,
            'position_cmd',
            self.__position_cmd_callback
        )

        self.get_logger().info("odrive service - velocity control")
        self.vel_cmd_service = self.create_service(
            VelocityControl,
            'velocity_cmd',
            self.__velocity_cmd_callback
        )

        self.get_logger().info("odrive msg - joint state")
        self.jointstate_pub = self.create_publisher(
            JointState,
            "Odrive_joint_topic",
            10
        )
        self.timer = self.create_timer(0.1, self.odrive_pub_callback)

    def __connect_odrive_callback(self, request: Trigger.Request, response: Trigger.Response)->None:
        self.get_logger().info("Loading parameters for Odrive...")
        self.cur_limit = self.get_parameter('cur_limit').get_parameter_value().double_value
        self.vel_limit = self.get_parameter('vel_limit').get_parameter_value().double_value
        self.calibration_current = self.get_parameter('calibration_current').get_parameter_value().double_value
        self.brake_resistor = self.get_parameter('brake_resistor').get_parameter_value().integer_value
        self.dc_max_negative_current = self.get_parameter('dc_max_negative_current').get_parameter_value().double_value
        self.pole_pairs = self.get_parameter('pole_pairs').get_parameter_value().integer_value
        self.torque_constant = self.get_parameter('torque_constant').get_parameter_value().double_value
        self.cpr = self.get_parameter('cpr').get_parameter_value().integer_value

        try:
            self.get_logger().info("Connecting to Odrive...")            
            # Find a connected ODrive (this will block until you connect one)
            self.get_logger().info(f'ODrive connected, ODrive bus voltage = {self.Node_odrive.vbus_voltage}')
            response.success=True
            response.message = f'Connected to {self.Node_odrive.serial_number}'

            self.Node_odrive.axis0.motor.config.current_lim = self.cur_limit
            self.Node_odrive.axis1.motor.config.current_lim = self.cur_limit
            self.Node_odrive.axis0.controller.config.vel_limit = self.vel_limit
            self.Node_odrive.axis1.controller.config.vel_limit = self.vel_limit
            self.Node_odrive.axis0.motor.config.calibration_current = self.calibration_current
            self.Node_odrive.axis1.motor.config.calibration_current = self.calibration_current
            self.Node_odrive.config.enable_brake_resistor = self.brake_resistor
            self.Node_odrive.config.dc_max_negative_current = self.dc_max_negative_current
            self.Node_odrive.axis0.motor.config.pole_pairs = self.pole_pairs
            self.Node_odrive.axis1.motor.config.pole_pairs = self.pole_pairs
            self.Node_odrive.axis0.motor.config.torque_constant = self.torque_constant
            self.Node_odrive.axis1.motor.config.torque_constant = self.torque_constant
            self.Node_odrive.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            self.Node_odrive.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            self.Node_odrive.axis0.encoder.config.cpr = self.cpr
            self.Node_odrive.axis1.encoder.config.cpr = self.cpr

            self.Node_odrive.axis0.trap_traj.config.vel_limit = 2.0
            self.Node_odrive.axis0.trap_traj.config.accel_limit = 1.2
            self.Node_odrive.axis0.trap_traj.config.decel_limit = 1.2
            self.Node_odrive.axis0.controller.config.inertia = 0.0

            if self.Node_odrive.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                # Calibrate motor and wait for it to finish
                self.get_logger().info("starting calibration...")
                self.Node_odrive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                self.Node_odrive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                while self.Node_odrive.axis0.current_state != AXIS_STATE_IDLE and \
                    self.Node_odrive.axis1.current_state != AXIS_STATE_IDLE:
                    time.sleep(0.1)

                self.Node_odrive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                self.Node_odrive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                self.get_logger().info("Odrive Ready!")
            else:
                self.get_logger().info("Odrive Ready!")
        except TimeoutError:
            response.success = False
            response.message = 'Timeout Error'
        except:
            response.success = False
            response.message = f'Unexpected error: {sys.exc_info()[0]}'
        return response
    
    def __request_state_callback(self, staterequest: AxisState.Request, stateresponse: AxisState.Response)->AxisState.Response: 
        if self.Node_odrive:
            self.get_logger().info(f'Odrive request state')
            match int(staterequest.axis):
                case 0:
                    self.Node_odrive.axis0.requested_state = staterequest.state
                    self.Node_odrive.axis0.watchdog_feed()
                    stateresponse.success = True
                    stateresponse.state = self.Node_odrive.axis0.current_state
                    stateresponse.message = f'Odrive State = {staterequest.state}'
                case 1:
                    self.Node_odrive.axis1.requested_state = staterequest.state
                    self.Node_odrive.axis1.watchdog_feed()
                    stateresponse.success = True
                    stateresponse.state = self.Node_odrive.axis1.current_state
                    stateresponse.message = f'Odrive State = {staterequest.state}'
                case other:
                    stateresponse.success = False
                    stateresponse.message = "Axis not exist"
        else:
            stateresponse.success = False
            stateresponse.message = f'ODrive not ready'
        return stateresponse

    def __control_modes_callback(self, controlrequest: AxisModes.Request, controlresponse: AxisModes.Response)->AxisModes.Response: 
        if self.Node_odrive:
            self.get_logger().info(f'Odrive control modes')
            self.__is_control_mode_service_called = True
            match int(controlrequest.axis):
                case 0:
                    self.Node_odrive.axis0.controller.config.input_mode = controlrequest.control_mode
                    self.__config_control_mode(controlrequest)
                    self.Node_odrive.axis0.watchdog_feed()
                    controlresponse.success = True
                    controlresponse.current_control_mode = self.Node_odrive.axis0.controller.config.input_mode
                    controlresponse.message = f'{self.__print_A_name(controlrequest.control_mode,constants.INPUT_MODES_LIST)}'
                case 1:
                    self.Node_odrive.axis1.controller.config.input_mode = controlrequest.control_mode
                    self.__config_control_mode(controlrequest)
                    self.Node_odrive.axis1.watchdog_feed()
                    controlresponse.success = True
                    controlresponse.current_control_mode = self.Node_odrive.axis1.controller.config.input_mode
                    controlresponse.message = f'{self.__print_A_name(controlrequest.control_mode,constants.INPUT_MODES_LIST)}'
                case other:
                    controlresponse.success = False
                    controlresponse.message = "Axis not exist"
        else:
            controlresponse.success = False
            controlresponse.message = f'ODrive not ready'
        return controlresponse
    
    def __position_cmd_callback(self, pos_cmd_request: PositionControl.Request, pos_cmd_response: PositionControl.Response)->PositionControl.Response:
        self.get_logger().info(f'Odrive position control')
        if self.__is_control_mode_service_called:
            match int(pos_cmd_request.axis):
                case 0:
                    self.Node_odrive.axis0.controller.input_pos = pos_cmd_request.turns
                    self.Node_odrive.axis0.watchdog_feed()
                    pos_cmd_response.success = True
                    pos_cmd_response.message = f'{str(pos_cmd_request.turns)}'
                case 1:
                    self.Node_odrive.axis1.controller.input_pos = pos_cmd_request.turns
                    self.Node_odrive.axis1.watchdog_feed()
                    pos_cmd_response.success = True
                    pos_cmd_response.message = f'{str(pos_cmd_request.turns)}'
        else:
            self.Node_odrive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.Node_odrive.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
            self.Node_odrive.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.Node_odrive.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ

            match int(pos_cmd_request.axis):
                case 0:
                    self.Node_odrive.axis0.controller.move_incremental(pos_cmd_request.turns, False)
                    self.Node_odrive.axis0.watchdog_feed()
                    pos_cmd_response.success = True
                    pos_cmd_response.message = f'{str(pos_cmd_request.turns)}'
                case 1:
                    self.Node_odrive.axis1.controller.move_incremental(pos_cmd_request.turns, False)
                    self.Node_odrive.axis1.watchdog_feed()
                    pos_cmd_response.success = True
                    pos_cmd_response.message = f'{str(pos_cmd_request.turns)}'
        return pos_cmd_response
    
    def __velocity_cmd_callback(self, vel_cmd_request: VelocityControl.Request, vel_cmd_response: VelocityControl.Response)->VelocityControl.Response:
        self.get_logger().info(f'Odrive velocity control')
        if self.__is_control_mode_service_called:
            match vel_cmd_request.axis:
                case 0:
                    self.Node_odrive.axis0.controller.input_vel = vel_cmd_request.turns_s
                    self.Node_odrive.axis0.watchdog_feed()
                    vel_cmd_response.success = True
                    vel_cmd_response.message = f'{str(vel_cmd_request.turns_s)}'
                case 1:
                    self.Node_odrive.axis1.controller.input_vel = vel_cmd_request.turns_s
                    self.Node_odrive.axis1.watchdog_feed()
                    vel_cmd_response.success = True
                    vel_cmd_response.message = f'{str(vel_cmd_request.turns_s)}'
        else:
            self.Node_odrive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.Node_odrive.axis0.controller.config.vel_ramp_rate = 0.5
            self.Node_odrive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.Node_odrive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.Node_odrive.axis1.controller.config.vel_ramp_rate = 0.5
            self.Node_odrive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.get_logger().info(f'vel_input')
            match int(vel_cmd_request.axis):
                case 0:
                    self.Node_odrive.axis0.controller.input_vel = vel_cmd_request.turns_s
                    self.Node_odrive.axis0.watchdog_feed()
                    vel_cmd_response.success = True
                    vel_cmd_response.message = f'{str(vel_cmd_request.turns_s)}'
                case 1:
                    self.Node_odrive.axis1.controller.input_vel = vel_cmd_request.turns_s
                    self.Node_odrive.axis1.watchdog_feed()
                    vel_cmd_response.success = True
                    vel_cmd_response.message = f'{str(vel_cmd_request.turns_s)}'

        return vel_cmd_response

    def odrive_pub_callback(self)->None:
        odrive_state = JointState()
        if self.Node_odrive:
            odrive_state.header.stamp = self.get_clock().now().to_msg()
            odrive_state.position = [self.Node_odrive.axis0.encoder.pos_estimate,
                            self.Node_odrive.axis1.encoder.pos_estimate]
            odrive_state.velocity = [self.Node_odrive.axis0.encoder.vel_estimate,
                            self.Node_odrive.axis1.encoder.vel_estimate]
            self.jointstate_pub.publish(odrive_state)
        else:
            self.get_logger().debug('ODrive not ready')

    def __print_A_name(self,name_index:int, list:list)->str:
        print(type(name_index))
        if int(name_index) >= 0 and int(name_index) <= 8:
            ans = str(list[name_index])
        else:
            ans = "error"
        return ans

    def __config_control_mode(self, controlrequest: AxisModes.Request)->None:
        if controlrequest.axis == 0:
            match controlrequest.control_mode:
                case 2: 
                    self.Node_odrive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                    self.Node_odrive.axis0.controller.config.vel_ramp_rate = controlrequest.vel_ramp_rate
                    self.Node_odrive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
                case 3:
                    self.Node_odrive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    self.Node_odrive.axis0.controller.config.input_filter_bandwidth = controlrequest.input_filter_bandwidth
                    self.Node_odrive.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
                case 5:
                    self.Node_odrive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    self.Node_odrive.axis0.trap_traj.config.vel_limit = controlrequest.trap_traj_vel_limit
                    self.Node_odrive.axis0.trap_traj.config.accel_limit = controlrequest.trap_traj_accel_limit
                    self.Node_odrive.axis0.trap_traj.config.decel_limit = controlrequest.trap_traj_decel_limit
                    self.Node_odrive.axis0.controller.config.inertia = controlrequest.trap_traj_inertia
                    self.Node_odrive.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
                case 6:
                    self.Node_odrive.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
                case other:
                    print("Error")
        elif controlrequest.axis == 1:
            match controlrequest.control_mode:
                case 2: 
                    self.Node_odrive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                    self.Node_odrive.axis1.controller.config.vel_ramp_rate = controlrequest.vel_ramp_rate
                    self.Node_odrive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
                case 3:
                    self.Node_odrive.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    self.Node_odrive.axis1.controller.config.input_filter_bandwidth = controlrequest.input_filter_bandwidth
                    self.Node_odrive.axis1.controller.config.input_mode = INPUT_MODE_POS_FILTER
                case 5:
                    self.Node_odrive.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                    self.Node_odrive.axis1.trap_traj.config.vel_limit = controlrequest.trap_traj_vel_limit
                    self.Node_odrive.axis1.trap_traj.config.accel_limit = controlrequest.trap_traj_accel_limit
                    self.Node_odrive.axis1.trap_traj.config.decel_limit = controlrequest.trap_traj_decel_limit
                    self.Node_odrive.axis1.controller.config.inertia = controlrequest.trap_traj_inertia
                    self.Node_odrive.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
                case 6:
                    self.Node_odrive.axis1.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
                case other:
                    print("Error")


                

def main(args=None):
    rclpy.init(args=args)
    node = OdriveNode("node_odrive_srv")
    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()