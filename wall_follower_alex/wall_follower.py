#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
# from scipy.optimize import curve_fit

from wall_follower.visualization_tools import VisualizationTools


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
		
        ## initializing the drive publisher and laserscan subscriber
        self.drive_pub = self.create_publisher(AckermannDriveStamped,self.DRIVE_TOPIC,10)
        self.laser_sub = self.create_subscription(LaserScan,self.SCAN_TOPIC,self.laser_callback,10)
        self.line_pub = self.create_publisher(Marker,"/laser",10)

        self.previous_error = 0.0
        self.Kp = 1.3
        self.Kd = 4.1
    def laser_callback(self,laserscan):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
    
        # wall tracer message
        distances = np.array(laserscan.ranges)
        # need to split up ranges intelligently so that the wall tracer will 
        # only trace the portion of wall that is immediately adjacent to it



        # length_wall = int(len(distances)/3)
        # middle = distances[6*length_wall//5:9*length_wall//5]


        # left = distances[length_wall//2:7*length_wall//5]
        # right = distances[3*length_wall//2:12*length_wall//5]
        # mask = left < 3*self.DESIRED_DISTANCE
        # mask2 = right < 3*self.DESIRED_DISTANCE


        # # left = left[mask]
        # # right = right[mask2]
        # angles = np.arange(laserscan.angle_min,laserscan.angle_max,laserscan.angle_increment)
        # angles_left = angles[length_wall//2:7*length_wall//5]
        # angles_right = angles[3*length_wall//2:12*length_wall//5]
        # # angles_left=angles_left[mask]
        # # angles_right=angles_right[mask2]
        length_wall = int(len(distances)/3)
        middle = distances[6*length_wall//5:9*length_wall//5]
        left = distances[length_wall//2:7*length_wall//5]
        right = distances[3*length_wall//2:12*length_wall//5]
        # left = left[left<3*self.DESIRED_DISTANCE]
        # right = right[right<3*self.DESIRED_DISTANCE]
        angles = np.arange(laserscan.angle_min,laserscan.angle_max,laserscan.angle_increment)
        angles_left = angles[length_wall//2:7*length_wall//5]
        angles_right = angles[3*length_wall//2:12*length_wall//5]

        # assert(len(angles_left)==len(left))
        # assert(len(angles_right==len(right)))
        if self.SIDE > 0: ##left
            x = right*np.cos(angles_right)
            y = right*np.sin(angles_right)
        else:
            x = left*np.cos(angles_left)
            y = left*np.sin(angles_left)
            
            # y = y[y<3*self.DESIRED_DISTANCE]
        # now, linear regression to get the wall (either left or right)
        A = np.vstack([x,np.ones(len(x))]).T
        slope,intercept = np.linalg.lstsq(A,y,rcond=None)[0]
        ## this function will plot the points of the wall adjacent to car
        new_y = slope*x+intercept
        VisualizationTools.plot_line(x,new_y,self.line_pub,frame="/laser")

        # self.get_logger().info('new_y[0]: %s' % new_y[0])

        ## controller also goes in here, will be the one that determines
        # steering angle

        error = self.DESIRED_DISTANCE - np.abs(new_y[len(left)//2])
        deriv = error - self.previous_error
        self.previous_error = error
        control = self.Kp*error + self.Kd*deriv
        ## driving message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = "laser"
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = min(self.VELOCITY,4.0)
        drive_msg.drive.steering_angle = -1*self.SIDE*min(control,0.34)
        self.drive_pub.publish(drive_msg)

def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
