from __future__ import print_function
import sys
import rclpy
import time
import odrive
from odrive.enums import *
from rclpy.node import Node
from std_srvs.srv import Trigger

class OdriveNode(Node):
    def __init__(self, name):
        super().__init__(name)
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
                ('connection_timeout',rclpy.Parameter.Type.INTEGER)
            ]
        )
        self.get_logger().info("Starting Odrive service...")
        self.connect_odrive_service = self.create_service(
            Trigger,
            'connect_odrive',
            self.connect_odrive_callback
        )

    def connect_odrive_callback(self, request: Trigger.Request, response: Trigger.Response)->None:
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
            self.Node_odrive = odrive.find_any(
                timeout=self.get_parameter(
                    'connection_timeout'
                ).get_parameter_value().integer_value
            )
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
        
        
def main(args=None):
    rclpy.init(args=args)
    node = OdriveNode("node_odrive_srv")
    rclpy.spin(node)
    node.destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()