#!/usr/bin/env python3
import numpy as np
import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

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
        self.declare_parameter('wall_topic', '/wall')
        self.declare_parameter('loss_topic','/loss')
        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.WALL_TOPIC = self.get_parameter('wall_topic').get_parameter_value().string_value 
        self.LOSS_TOPIC = self.get_parameter('loss_topic').get_parameter_value().string_value
        # Based on the right or left side, the car will only 
        # account for scans in the cone between these angles
        # side_min_angle_magnitude : Radians, set >= 0
        # side_max_angle_magnitude : Radians, set <= 135 
        self.side_min_angle_magnitude = -45* (np.pi/180)
        self.side_max_angle_magnitude = 135 * (np.pi/180)

        # Another parameter you can set is the distance at which a scan is disregarded (Look-Ahead Distance) 
        # max_scan_distance : meters
        self.max_scan_distance = 3*self.DESIRED_DISTANCE

        # For Visualization
        self.visualize_line = True
        
        # Needed for PID
        self.previous_distance_error = 0
        self.previous_slope_error = 0
        self.previous_time = 0

        # Loss arrays
        self.loss_array = []
    
        # Publishers
        self.wall_visual_publihser = self.create_publisher(Marker, self.WALL_TOPIC, 10)
        self.drive_input_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC,10)
        self.loss_publisher = self.create_publisher(Float32,self.LOSS_TOPIC,10)
        # Subscription 
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_callback, 10)
        # self.total_loss = 0
    
    # Subscription Callback Function (Scan Processing, Feed Errors into Controller, & Send Drive Command)
    def laser_callback(self, laser_scan):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        current_time = (laser_scan.header.stamp.nanosec / 1e9)
        # self.get_logger().info(str(self.VELOCITY))
        num_points = len(laser_scan.ranges)
        distances = np.array(laser_scan.ranges)
        angles = np.linspace(laser_scan.angle_min, laser_scan.angle_max, num_points)
        scan_polar_vectors = np.vstack((distances, angles))
        
        # Filter Points based on Side & Angles
        if self.SIDE == 1:
            scan_polar_vectors = scan_polar_vectors[:, (scan_polar_vectors[1,:]<= self.side_max_angle_magnitude) & (scan_polar_vectors[1,:]>= self.side_min_angle_magnitude)]
        else:
            scan_polar_vectors = scan_polar_vectors[:, (-scan_polar_vectors[1,:] <= self.side_max_angle_magnitude) & (-scan_polar_vectors[1,:] >= self.side_min_angle_magnitude)]
        
        # Filter Points based on Look-Ahead Distance
        scan_polar_vectors = scan_polar_vectors[:, scan_polar_vectors[0, :] <= self.max_scan_distance]
            
        # Find X,Y points (+Visualize), Gather PID output, Feed to Command Input
        if scan_polar_vectors.size == 0:
            output_steering_angle = 0
            velocity = self.VELOCITY
        else:
            xs = scan_polar_vectors[0,:]*np.cos(scan_polar_vectors[1,:])
            ys = scan_polar_vectors[0,:]*np.sin(scan_polar_vectors[1,:])

            best_model, best_inliers = self.ransac(xs, ys, sample_size=len(xs))

            # Extracting slope and intercept from the best model
            slope, intercept = best_model
            # slope, intercept = np.polyfit(xs, ys, 1)

            if self.SIDE == 1:
                intercept  -= self.DESIRED_DISTANCE
            else: 
                intercept += self.DESIRED_DISTANCE
            
            # Visualize
            if self.visualize_line:
                x = xs
                y = np.array([slope * xi + intercept for xi in x])
                VisualizationTools.plot_line(x, y, self.wall_visual_publihser, frame="/laser")

            errors = [intercept,slope]

            #PID control
            output_steering_angle = self.PID_controler(errors, current_time)
            velocity = self.VELOCITY * max(0.5, 0.1 + output_steering_angle**2)
            
        ## calculating loss
        loss = np.abs(self.DESIRED_DISTANCE - min(y))
        loss_msg = Float32()
        loss_msg.data = loss
        self.loss_array.append(loss)
        self.loss_publisher.publish(loss_msg)

        msg = AckermannDriveStamped()
        msg.drive.speed = velocity 
        msg.drive.steering_angle = float(output_steering_angle)
        self.drive_input_publisher.publish(msg)
        # self.get_logger().info()
    def PID_controler(self, error, curr_time):

        # Assuming error is distance to wall
        # error[0] is distance, error[1] is slope
        KP_Distance = 0.5
        KD_Distance = 0.1
        KI_Distance = 0

        KP_Slope = 1
        KD_Slope = 0.3
        KI_Slope = 0

        dt = curr_time - self.previous_time
        de_distance = error[0] - self.previous_distance_error
        de_slope = error[1] - self.previous_slope_error
        error_dot_distance = de_distance/dt
        error_dot_slope = de_slope/dt

        self.previous_time = curr_time
        self.previous_distance_error = error[0]
        self.previous_slope_error = error[1]

        return_value = (KP_Distance*error[0] + KP_Slope*error[1] + KD_Distance*error_dot_distance + KD_Slope*error_dot_slope)/2
        if return_value < -0.34:
            return_value = -0.34
        if return_value > 0.34:
            return_value = 0.34
        
        # self.get_logger().info(f'PD: {KP_Distance*error[0]} PS:{KP_Slope*error[1]} DD:{KD_Distance*error_dot_distance} DS:{KD_Slope*error_dot_slope}')
        # self.get_logger().info(f'{return_value}')
        return (KP_Distance*error[0] + KP_Slope*error[1] + KD_Distance*error_dot_distance + KD_Slope*error_dot_slope)/2

    
    def ransac(self,X, y, n_iterations=100, sample_size=20, inlier_threshold=1.0):
        best_model = None
        best_inliers = None
        max_inliers = 0

        for _ in range(n_iterations):
            # Step 1: Randomly sample a subset of points
            random_indices = random.sample(range(len(X)), sample_size)
            X_sampled = X[random_indices]
            y_sampled = y[random_indices]

            # Step 2: Fit a model to the sampled points
            model = np.polyfit(X_sampled, y_sampled, 1)  # Linear regression model, adjust degree as needed

            # Step 3: Calculate inliers based on the model
            residuals = np.abs(y - np.polyval(model, X))
            inliers = np.where(residuals < inlier_threshold)[0]

            # Step 4: Check if the current model has more inliers
            if len(inliers) > max_inliers:
                max_inliers = len(inliers)
                best_model = model
                best_inliers = inliers

        return best_model, best_inliers




def main():
    
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    