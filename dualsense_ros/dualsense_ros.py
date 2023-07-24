import rclpy
from rclpy.node import Node

import yaml

from pydualsense import *
from sensor_msgs.msg import Joy

class DualsenseRosDriver(Node):
    def __init__(self): #TODO move node name to constuctor
        super().__init__('dualsense_ros_node')
        self._dualsense = pydualsense()

        # Loading configuration files for button mapping
        with open("/home/s/ros2_ws/src/dualsense_ros/config/button_mapping.yaml") as config:
            try:
                self._button_mapping = yaml.safe_load(config)
            except yaml.YAMLError as err:
                self.get_logger().warn("Failed to load button configuration")
        config.close()
        
        with open("/home/s/ros2_ws/src/dualsense_ros/config/joystick_mapping.yaml") as config:
            try:
                self._joystick_mapping = yaml.safe_load(config)
            except yaml.YAMLError as err:
                self.get_logger().warn("Failed to load joystick configuration")
        config.close()
                
        # Base joy msg creation
        self._joy_msg  = Joy()
        self._joy_msg.buttons = [0] * len(self._button_mapping)
        self._joy_msg.axes = [.0] * len(self._joystick_mapping)

        
    
    def initialize(self):
        try:
            self._dualsense.init()
            self.joy_publisher_timer = self.create_timer(0.05,self.joy_publisher_timer_callback) #TODO parametrize timer
            self.joy_publisher = self.create_publisher(Joy,"joy",10)
        except Exception as e:
            self.get_logger().error('Failed to initialize node: %r' % (e,))
            raise IOError
        else:
            self.get_logger().info('Node initialized successfully')
        self.get_logger().info('"{0}"'.format(self._button_mapping))
               
    def joy_publisher_timer_callback(self):
        
        self._joy_msg._buttons[self._button_mapping['SQUARE']]    = self._dualsense.state.square
        self._joy_msg._buttons[self._button_mapping['TRIANGlE']]  = self._dualsense.state.triangle
        self._joy_msg._buttons[self._button_mapping['CIRCLE']]    = self._dualsense.state.circle
        self._joy_msg._buttons[self._button_mapping['CROSS']]     = self._dualsense.state.cross
        
        self._joy_msg._buttons[self._button_mapping['DPADUP']]    = self._dualsense.state.DpadUp
        self._joy_msg._buttons[self._button_mapping['DPADDOWN']]  = self._dualsense.state.DpadDown
        self._joy_msg._buttons[self._button_mapping['DPADLEFT']]  = self._dualsense.state.DpadLeft
        self._joy_msg._buttons[self._button_mapping['DPADRIGHT']] = self._dualsense.state.DpadRight
        
        self._joy_msg._buttons[self._button_mapping['L1']]        = self._dualsense.state.L1
        self._joy_msg._buttons[self._button_mapping['L2']]        = self._dualsense.state.L2
        self._joy_msg._buttons[self._button_mapping['L2PRESSED']] = self._dualsense.state.L2Btn
        self._joy_msg._buttons[self._button_mapping['L3']]        = self._dualsense.state.L3
        
        self._joy_msg._buttons[self._button_mapping['R1']]        = self._dualsense.state.R1
        self._joy_msg._buttons[self._button_mapping['R2']]        = self._dualsense.state.R2
        self._joy_msg._buttons[self._button_mapping['R2PRESSED']] = self._dualsense.state.R2Btn
        self._joy_msg._buttons[self._button_mapping['R3']]        = self._dualsense.state.R3
        
        self._joy_msg._buttons[self._button_mapping['SHARE']]     = self._dualsense.state.share
        self._joy_msg._buttons[self._button_mapping['OPTIONS']]   = self._dualsense.state.options
        self._joy_msg._buttons[self._button_mapping['PS']]        = self._dualsense.state.ps
        self._joy_msg._buttons[self._button_mapping['MIC']]       = self._dualsense.state.micBtn

        self._joy_msg.axes[self._joystick_mapping['RX']]          = self._dualsense.state.RX
        self._joy_msg.axes[self._joystick_mapping['RY']]          = self._dualsense.state.RY
        self._joy_msg.axes[self._joystick_mapping['LX']]          = self._dualsense.state.LX
        self._joy_msg.axes[self._joystick_mapping['LY']]          = self._dualsense.state.LY

        
        self.joy_publisher.publish(self._joy_msg)
    
    def shutdown(self, close_dualsense:bool = False):
        if close_dualsense:
            try:
                self._dualsense.close()
            except Exception as e:
                self.get_logger().warning("Failed to close Dualsense: %r" %(e,))
        self.get_logger().info("Shutting down.....")
        self.destroy_node()
        
            
        
