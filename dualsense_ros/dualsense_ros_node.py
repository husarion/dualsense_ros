import rclpy
from rclpy import logging

from dualsense_ros.dualsense_ros import DualsenseRosDriver

def main():
    rclpy.init()
    dualsense_ros_driver_node = DualsenseRosDriver()
    try:
        dualsense_ros_driver_node.initialize()
        rclpy.spin(dualsense_ros_driver_node)
    except IOError as e:
        logging.get_logger("SHUTDOWN").error('Shutting Down: %r' % (e,))
        dualsense_ros_driver_node.shutdown()
        rclpy.shutdown()
    except Exception as e:
        logging.get_logger("SHUTDOWN").error('Shutting Down: %r' % (e,))
        dualsense_ros_driver_node.shutdown(close_dualsense=True)
        rclpy.shutdown()
if __name__ == '__main__':
    main()
    
    