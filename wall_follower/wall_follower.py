#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

from wall_follower.visualization_tools import VisualizationTools

from wall_follower.pure_pursuit import pure_pursuit



class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
	# TODO: Initialize your publishers and subscribers here
        ##Subscribers
        self.lidar_sub=self.create_subscription(LaserScan, self.SCAN_TOPIC, self.lidar_callback, 10)

        ##Publishers
        self.drive_pub=self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)




#####Visualization code
        WALL_TOPIC = "/wall"

        # a publisher for our line marker
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.lidar_callback, 10)
        self.line_pub = self.create_publisher(Marker, WALL_TOPIC, 1)

        # a subscriber to get the laserscan data
        

    
       

#################

  



    def lidar_callback(self, msg): ##All things Ackerman
        
        ###IRENE CODE, allows for tests to update 
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        ###
        
        
        angle, x, y, m, b =pure_pursuit(msg, self.SIDE, self.VELOCITY, self.DESIRED_DISTANCE) 
        

        output=AckermannDriveStamped()
        output.drive.steering_angle= angle     # desired virtual angle (radians) left is positive
        # output.drive.steering_angle_velocity= 0 # desired rate of change (radians/s) absolute quantity, 0 means changing as fast as possible
        output.drive.speed= self.VELOCITY                   # desired forward speed (m/s)
        # output.drive.acceleration =0          # desired acceleration (m/s^2), 0 means change as fast as possible
        # output.drive.jerk =0                  # desired jerk (m/s^3), 0 means change as fast as possible
        output.header.stamp = self.get_clock().now().to_msg()
        # output.header.frame_id = 0 #'world' 0 is no frame, 1 is global

        self.drive_pub.publish(output) #publishes message, 
        #self.get_logger().info(str(m))

        VisualizationTools.plot_line(x, x*m+b, self.line_pub, frame="/laser")


def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
