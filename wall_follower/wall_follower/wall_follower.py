#!/usr/bin/env python3
import numpy as np
import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
# from visualization_msgs.msg import Marker


# from wall_follower.visualization_tools import VisualizationTools


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
        self.declare_parameter('safety_drive_topic', '/follow_drive')

        # Fetch constants from the ROS parameter server
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SAFETY_DRIVE_TOPIC = self.get_parameter('safety_drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.WALL_TOPIC = self.get_parameter('wall_topic').get_parameter_value().string_value 

        # Based on the right or left side, the car will only 
        # account for scans in the cone between these angles
        # side_min_angle : Radians, set >= 0
        # side_max_angle : Radians, set <= 135 
        self.side_min_angle = -45* (np.pi/180)
        self.side_max_angle = 135 * (np.pi/180)

        # Another parameter you can set is the distance at which a scan is disregarded (Look-Ahead Distance) 
        # max_scan_distance : meters
        self.max_scan_distance = 3*self.DESIRED_DISTANCE

        # For Visualization
        self.visualize_line = True
        
        # Needed for PID
        self.prev_dist_error = 0
        self.prev_angle_error = 0
        self.prev_time = 0
    
        # Publishers
        # self.wall_visual_publisher = self.create_publisher(Marker, self.WALL_TOPIC, 10)
        self.drive_input_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10) # Set to self.DRIVE_TOPIC to circumvent the safety controller

        # Subscribtion 
        self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_callback, 10)

    
    # Subscription Callback Function (Scan Processing, Feed Errors into Controller, & Send Drive Command)
    def laser_callback(self, laser_scan):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        current_time = (laser_scan.header.stamp.nanosec / 1e9)
        num_points = len(laser_scan.ranges)
        distances = np.array(laser_scan.ranges)
        angles = np.linspace(laser_scan.angle_min, laser_scan.angle_max, num_points)
        scan_vectors = np.vstack((distances, angles))
        
        # Filter Points based on Side & Angles
        if self.SIDE == 1:
            scan_vectors = scan_vectors[:, (scan_vectors[1,:]<= self.side_max_angle) & (scan_vectors[1,:]>= self.side_min_angle)]
        else:
            scan_vectors = scan_vectors[:, (-scan_vectors[1,:] <= self.side_max_angle) & (-scan_vectors[1,:] >= self.side_min_angle)]
        
        # Filter Points based on Look-Ahead Distance
        scan_vectors = scan_vectors[:, scan_vectors[0, :] <= self.max_scan_distance]
            
        # Find X,Y points (+Visualize), Gather PID output, Feed to Command Input
        if scan_vectors.size == 0:
            output_steering_angle = 0
            velocity = self.VELOCITY
        else:
            xs = scan_vectors[0,:]*np.cos(scan_vectors[1,:])
            ys = scan_vectors[0,:]*np.sin(scan_vectors[1,:])

            best_model, best_inliers = self.ransac(xs, ys, sample_size=len(xs))

            # Extracting slope and intercept from the best model
            slope, intercept = best_model
            # slope, intercept = np.polyfit(xs, ys, 1)

            if self.SIDE == 1:
                intercept  -= self.DESIRED_DISTANCE
            else: 
                intercept += self.DESIRED_DISTANCE
            
            # Visualize
            # if self.visualize_line:
            #     x = xs
            #     y = np.array([slope * xi + intercept for xi in x])
            #     VisualizationTools.plot_line(x, y, self.wall_visual_publisher, frame="/laser")

            errors = [intercept,slope]

            #PID control
            output_steering_angle = self.PID_controler(errors, current_time)
            velocity = self.VELOCITY
            
        msg = AckermannDriveStamped()
        msg.drive.speed = velocity
        msg.drive.steering_angle = output_steering_angle
        self.drive_input_publisher.publish(msg)

    def PID_controler(self, error, curr_time):

        # Assuming error is distance to wall
        # error[0] is distance, error[1] is slope
        Kp_dist = 0.5
        Kd_dist = 0.1
        Ki_dist = 0

        Kp_slope = 1
        Kd_slope = 0.3
        Ki_slope = 0

        dt = curr_time - self.prev_time
        de_dist = error[0] - self.prev_dist_error
        de_slope = error[1] - self.prev_angle_error
        de_dt_dist = de_dist/dt
        de_dt_slope = de_slope/dt

        self.prev_time = curr_time
        self.prev_dist_error = error[0]
        self.prev_angle_error = error[1]

        return_value = (Kp_dist*error[0] + Kp_slope*error[1] + Kd_dist*de_dt_dist + Kd_slope*de_dt_slope)/2
        if return_value < -0.34:
            return_value = -0.34
        if return_value > 0.34:
            return_value = 0.34
        
        self.get_logger().info(f'{return_value}')
        return (Kp_dist*error[0] + Kp_slope*error[1] + Kd_dist*de_dt_dist + Kd_slope*de_dt_slope)/2

    
    def ransac(self,X, y, n_iters=100, sample_size=20, inlier_threshold=1.0):
        best_model = None
        best_inliers = None
        max_inliers = 0

        for _ in range(n_iters):
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
    
 