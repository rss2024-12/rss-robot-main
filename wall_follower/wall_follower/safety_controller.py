import numpy as np
import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
# from visualization_msgs.msg import Marker

class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters to make them available for use
        # self.declare_parameter("scan_topic", "default")
        # self.declare_parameter("drive_topic", "default")
        # self.declare_parameter("side", "default")
        # self.declare_parameter("velocity", "default")
        # self.declare_parameter("desired_distance", "default")
        # self.declare_parameter('wall_topic', '/wall')
        # self.declare_parameter('safety_drive_topic', '/follow_drive')

        # Fetch constants from the ROS parameter server
        # self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        # self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        # self.SAFETY_DRIVE_TOPIC = self.get_parameter('safety_drive_topic').get_parameter_value().string_value
        # self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        # self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        # self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        # self.WALL_TOPIC = self.get_parameter('wall_topic').get_parameter_value().string_value 

        # Based on the right or left side, the car will only 
        # account for scans in the cone between these angles
        # side_min_angle_magnitude : Radians, set >= 0
        # side_max_angle_magnitude : Radians, set <= 135 
        self.min_angle_magnitude = -15* (np.pi/180)
        self.max_angle_magnitude = 15 * (np.pi/180)
        self.latest_drive_msg = AckermannDriveStamped()

        # Another parameter you can set is the distance at which a scan is disregarded (Look-Ahead Distance) 
        # max_scan_distance : meters
        self.min_scan_distance = 0.5*2
        self.is_stopped = False
    
        # Publishers
        self.drive_input_publisher = self.create_publisher(AckermannDriveStamped, "/vesc/low_level/input/safety", 10)

        # Subscribtion 
        self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.create_subscription(AckermannDriveStamped, "/vesc/high_level/output", self.drive_callback, 10)

    def laser_callback(self, laser_scan:LaserScan):
        distances = np.array(laser_scan.ranges)
        min_scan_index = int(len(distances)/2 + (self.min_angle_magnitude/laser_scan.angle_increment))
        max_scan_index = int(len(distances)/2 + (self.max_angle_magnitude/laser_scan.angle_increment))
        current_min_distance = np.min(distances[min_scan_index:max_scan_index])

        if current_min_distance <= self.min_scan_distance:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.drive_input_publisher.publish(msg)
            stop_distance = current_min_distance
            if not self.is_stopped:
                self.get_logger().info(f"STOP")
                self.get_logger().info(f"{current_min_distance}")
                self.is_stopped = True

        elif current_min_distance > self.min_scan_distance:
            self.is_stopped = False
            # self.drive_input_publisher.publish(self.latest_drive_msg)
            self.get_logger().info(f"NOT STOPPED")
            self.get_logger().info(f"{current_min_distance}")
        
    def drive_callback(self, drive_msg): # Remove once we have the actual bot. Since the mux will control which messages to publish
        self.latest_drive_msg = drive_msg


def main():
    
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
 