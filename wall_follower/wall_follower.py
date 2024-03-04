#!/usr/bin/env python3
import numpy as np
import rclpy
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

from wall_follower.visualization_tools import *


class WallFollower(Node):

    WALL_TOPIC = "/wall"
    FRONT_WALL_TOPIC ="/front_wall"
    BACK_WALL_TOPIC = "/back_wall"
    WALL_OFFSET_TOPIC = "/wall_offset"
    TURN_RADIUS = 1

    def __init__(self):
        super().__init__("wall_follower")
        # Declaring parameters to make them available for use
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
		
	    # Initialize publishers and subscribers
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.wall_publisher = self.create_publisher(Marker, self.WALL_TOPIC, 1)
        self.front_wall_publisher = self.create_publisher(Marker, self.FRONT_WALL_TOPIC, 1)
        self.back_wall_publisher = self.create_publisher(Marker, self.BACK_WALL_TOPIC, 1)
        self.wall_offset_publisher = self.create_publisher( Float32, self.WALL_OFFSET_TOPIC, 10)
        
        self.scan_subscription = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)

        self.last_angle = 0
        self.last_error = 0
        self.integrated_error = 0


    def scan_callback(self, scan_msg):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        
        # Computing relevant values and arrays for rest of method
        sample_size = len(scan_msg.ranges)
        front_index = sample_size//2
        view_range_start = scan_msg.angle_min
        view_range_end = scan_msg.angle_max
        view_range = view_range_end - view_range_start
        angles = np.linspace(view_range_start, view_range_end, sample_size)
        
        # Creating scan data array to be given to 'relevat_samples'
        # TODO: this can be changed, including the revelevant samples method
        scan_data = np.array([angles, scan_msg.ranges])

        # Use the relevant samples to gather relevant data for the front, back and specified side
        (relevant_data_polar, relevant_data_front_polar, relevant_data_back_polar) = self.relevant_samples(scan_data, self.SIDE)

        # USED DURING TESTING
        # if not relevant_data_polar.any():
        #     pass

        # Converting to cartesian and finding linear fit
        relevant_data_cartesian = self.polar_to_cartesian(relevant_data_polar)
        wall_m, wall_y_offset = self.linear_fit(relevant_data_cartesian)

        # Creating lines for visualization
        x_viz = np.linspace(-2., 2., num=20)
        y_viz = wall_m*x_viz + wall_y_offset
        VisualizationTools.plot_line(x_viz, y_viz, self.wall_publisher, frame="/laser")

        # Checking to see if the front or back of the car are 'good'
        front_good = False
        back_good = False
        if len(relevant_data_front_polar[1]) > 2:
            
            front_good = True
            relevant_data_front_cartesian = self.polar_to_cartesian(relevant_data_front_polar)
            front_wall_m, front_wall_y_offset = self.linear_fit(relevant_data_front_cartesian)

            front_x_wall = np.linspace(1., 4., num=16)
            front_y_wall = front_wall_m*front_x_wall + front_wall_y_offset

            VisualizationTools.plot_line(front_x_wall, front_y_wall, self.front_wall_publisher, frame="/laser")

        if len(relevant_data_back_polar[1]) > 2:
            
            back_good = True
            relevant_data_back_cartesian = self.polar_to_cartesian(relevant_data_back_polar)
            back_wall_m, back_wall_y_offset = self.linear_fit(relevant_data_back_cartesian)

            back_x_wall = np.linspace(-4., -1., num=16)
            back_y_wall = back_wall_m*back_x_wall + back_wall_y_offset

            VisualizationTools.plot_line(back_x_wall, back_y_wall, self.back_wall_publisher, frame="/laser")
        

        wall_offset = wall_y_offset + math.atan(wall_m)
        # self.get_logger().info('Wall offset: %s = %s + %s' % (wall_offset,  wall_y_offset, math.atan(wall_m)))
        
        Tc = 1.2
        Kp = 0.2
        Ki = 2*Kp/Tc    *0
        Kd = Kp*Tc/8      *0

        wall_offset_error = wall_offset - self.DESIRED_DISTANCE*self.SIDE
        wall_offset_error_dt = (wall_offset_error - self.last_error)
        self.integrated_error += wall_offset_error

        wheel_angle = wall_offset_error*Kp + wall_offset_error_dt*Kd + Ki*self.integrated_error

        # wheel_angle = 100*(wall_y_offset - self.DESIRED_DISTANCE*self.SIDE)

        # # checking to see if any of the ranges are less than the turn radius plus the desired distance
        # range_checker = False
        # for i in range(front_index-5, front_index+5):
        #     if scan_msg.ranges[i] < self.TURN_RADIUS + self.DESIRED_DISTANCE:
        #         checker = True

        # checker_side_1 = []
        # for s in scan_msg.ranges[front_index:(front_index*3)//4]:
        #     checker_side_1.append(s < 1.5*(self.TURN_RADIUS + self.DESIRED_DISTANCE))

        # checker_side_minus_1 = []
        # for s in scan_msg.ranges[front_index//4:front_index]:
        #     checker_side_minus_1.append(s < 1.5*(self.TURN_RADIUS + self.DESIRED_DISTANCE))

        # # Replaces If statment
        # if range_checker:
        #     if (self.SIDE == 1 and all(checker_side_1)) or (self.SIDE == -1 and all(checker_side_minus_1)):
        #         wheel_angle = -100.0 * self.SIDE
        #         self.integrated_error = 0

        
        # Original If statement
        if any([scan_msg.ranges[i] < self.TURN_RADIUS + self.DESIRED_DISTANCE for i in range(front_index-5, front_index+5)]):
            if self.SIDE == 1 and all([s < (self.TURN_RADIUS + self.DESIRED_DISTANCE)*1.5 for s in scan_msg.ranges[front_index:(front_index * 3)//4]]) or self.SIDE == -1 and all([s < (self.TURN_RADIUS + self.DESIRED_DISTANCE)*1.5 for s in scan_msg.ranges[front_index//4:front_index]]):
                wheel_angle = -100.0 * self.SIDE
                self.integrated_error = 0
        
        drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = self.get_clock().now().to_msg()
        # drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = wheel_angle
        # drive_msg.drive.steering_angle_velocity = float
        drive_msg.drive.speed = self.VELOCITY #* (0.75 + 0.25 * 1/(math.sqrt(abs(wheel_angle)+1)))
        # drive_msg.drive.acceleration = float
        # drive_msg.drive.jerk = float

        self.drive_publisher.publish(drive_msg)

        wall_offset_msg = Float32()
        wall_offset_msg.data = wall_offset_error
        self.wall_offset_publisher.publish(wall_offset_msg)

        self.last_error = wall_offset_error

    # Given data that includes x and y coordinates, generate a slope and y-intercept for line fit
    def linear_fit(self, data):
        x = data[0]
        y = data[1]
        A = np.vstack([x, np.ones(len(x))]).T
        m, b = np.linalg.lstsq(A, y, rcond=None)[0]
        return m, b
    
    # Given data that includes polar coordinates, convert them to cartesian 
    def polar_to_cartesian(self, data):
        theta = data[0]
        r = data[1]
        x = r*np.cos(theta)
        y = r*np.sin(theta)
        return np.array([x, y])
    
    # Given the scan data and side to follow, return relevant samples of the side as well as in front and behind
    def relevant_samples(self, samples, side):
        samples_pairwise = samples.T
        # self.get_logger().info('samples_pairwise: "%s"' % samples_pairwise)

        # here we are 
        if side == 1:
            # self.get_logger().info('taking LHS samples for self.SIDE == 1 == "%s"' % (self.SIDE,))
            out_samples = samples_pairwise[len(samples_pairwise)//2:]
        elif side == -1:
            # self.get_logger().info('taking RHS samples for self.SIDE == -1 == "%s"' % (self.SIDE,))
            out_samples = samples_pairwise[:len(samples_pairwise)//2]
        else:
            # self.get_logger().info('taking MID samples for self.SIDE == 0 == "%s"' % (self.SIDE,))
            out_samples = samples_pairwise[2*len(samples_pairwise)//5:3*len(samples_pairwise)//5]

        # original code
        closest_distance = (min([s[1] for s in out_samples]))
        furthest_distance_to_consider = (closest_distance +  self.DESIRED_DISTANCE)*2
        out_samples = np.array([s for s in out_samples if s[1] <= furthest_distance_to_consider])
        closest_distance_index = np.argmin(out_samples.T[1])

        # # finding closest distance
        # distances = []
        # for i in out_samples:
        #     distances.append(i[1])
        # closest_distance = min(distances)

        # # determining the furthest distance to consider
        # furthest_distance_to_consider = (closest_distance+self.DESIRED_DISTANCE)*2

        # # finding the distances that are less than the furthest distance just calculated
        # out_samples = []
        # for i in out_samples:
        #     if i[1] <= furthest_distance_to_consider:
        #         out_samples.append(i)
        # out_samples = np.array(out_samples)

        # closest_distance_index = np.argmin(out_samples.T[1])
        
        # determining the front and back samples based on the specified side
        if side == 1:
            out_samples_front = out_samples[:closest_distance_index]
            out_samples_back = out_samples[closest_distance_index:]
        elif side == -1:
            out_samples_front = out_samples[closest_distance_index:]
            out_samples_back = out_samples[:closest_distance_index]

        
        return (out_samples.T, out_samples_front.T, out_samples_back.T)
        

def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
