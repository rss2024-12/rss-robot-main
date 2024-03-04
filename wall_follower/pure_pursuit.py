import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from wall_follower.visualization_tools import VisualizationTools





def pure_pursuit(msg, flag, speed, d):
    """
    This file is meant to intake an array (LASERSCAN), and then return the values that will be used
    To generate the Ackermann inputs, this file will repeatedly be called within
    the node callback method in wall_follower.py

    Input: msg: array of lidar coordinates, of the type LaserScan
            flag: 1 if left, -1 if right
            speed: how fast we want the car to move, should be given by params
            d:ideal distance to maintain from wall

    Outputs steering angle
    

    """
    new_msg=range_splice(msg, flag) #returns useful array
    look_ahead=speed_to_L1(speed) #returns look ahead distance
    
    lidar_scan=scan_LIDAR(msg,new_msg,look_ahead,11,flag) #gives us distance and angle to wall, as well as its slope

    scan_angle=lidar_scan["angle_to_point"]
    fwall_distance=lidar_scan["average_point"]
    
    x=lidar_scan["x"]
    y=lidar_scan["y"]
    reg_m=lidar_scan["regression_m"]
    reg_b=lidar_scan["b"]

    (eta, L1) = from_wall_to_desired(fwall_distance, scan_angle, d, reg_m, flag)

    return (input_angle(eta, L1, 0.3),x,y,reg_m,reg_b) #apparently the distance is 0.3m



def range_splice(msg, flag):
    """
    Depending on the side, this function will give the relevant range of lidar scans
    
    

    """
    
   
    array=np.array(msg.ranges)
   

    rad_neg=round((-math.pi/2-msg.angle_min)/msg.angle_increment) #gives associated position of 90 degrees
    rad_pos=round((math.pi/2-msg.angle_min)/msg.angle_increment) #gives associated position of -90 degrees 
    if flag==1:#if following the left wall, we want the second half of angle array
        temp_array=array[len(array)//2:rad_pos] #starts half way through the array
        new_array=temp_array[::-1]#flips the list so we start with the angles closest to the 
    else: #otherwise we want the right 
        new_array=array[rad_neg:(len(array)+1)//2]
    return new_array


def speed_to_L1(speed):
    """
    This function takes an input speed(m/s), and returns how far ahead you need to look, i.e. L1 distance in m
    returns L1 distance
    """
    if speed < 1.34 :
        return 3
    elif 1.34 <= speed < 5.36:
        return 2.24*speed
    else:
        return 12
    


def scan_LIDAR(msg, LIDAR, look_ahead, X, flag):
    """
    This function scans through LIDAR, once it finds points at the proper distance
    uses an average of X points to make sure small errors don't cause 
    it will return the distance of the average point, as well as the slope of the regression of X points
    Input: msg:original scan
            LIDAR: only the segment we want, from splic 
            lookahead distance, 
            X points (how many we take the average of (num))
            flag: left or right


    Output: fwall_distance (average distance to wall)
            wall_angle (angle of average point )
            m slope of wall approx.
    """
    if flag==1: #implies left side scan
        start_angle=math.pi/2
    else:# implies right side scan
        start_angle=-math.pi/2 
   
    angle_increment=msg.angle_increment
    
    
    marker=0
    for i in range(len(LIDAR) - X + 1):
        # Calculate the average of the next X distances
        average_distance = np.mean(LIDAR[i:i+X])
        # Check if the average distance is greater than the threshold
        if average_distance >= look_ahead:
            subset_distances = LIDAR[i:i+X]
            subset_angles=[start_angle - i * (angle_increment*flag) -j*(angle_increment*flag) for j in range(X)] #increases angles if right side scan, decreases if left
            # Calculate x and y coordinates of points

            x_coords = subset_distances*np.cos(subset_angles) #x coord should always be positive
            y_coords = subset_distances*np.sin(subset_angles)#flag makes y negative if right side scan
            # Reshape x coordinates for regression
            marker=1 #this will tell me if i need to rebind the scan to the very end or not
            break
           
           
    if marker==0: #meaning we have not entered the loop
        subset_distances = LIDAR[i:i+X]
        subset_angles=[start_angle - i * (angle_increment*flag) -j*(angle_increment*flag) for j in range(X)]
        x_coords = subset_distances*np.cos(subset_angles) #x coord should always be positive
        y_coords = subset_distances*np.sin(subset_angles)#flag makes y negative if right side scan

    # Perform linear regression
    
    
    coefficients = np.polyfit(x_coords, y_coords, 1)
    m, b = coefficients
    # Determine turn direction
    
    # Calculate average point
    avg_x, avg_y = (np.mean(x_coords), np.mean(y_coords))
    point_distance=math.sqrt(avg_x**2+avg_y**2)#distance to average point
    point_angle=math.atan(avg_y/avg_x)# angle to average point

    #this code gets the wall estimation
    
    
    return {
        "x": x_coords,#shift to pass
        "y": y_coords,
        "average_point": point_distance, #returns the distance,and the angle
        "angle_to_point": point_angle,
        "regression_m": m,
        "b": b} #returns the slope of regression




def from_wall_to_desired(fwall_distance, scan_angle, d, m, flag):
    """
    Input of scan_angle should come from average point of points set
    Input of fwall_distance should come from average point of points set
    d = desired distance from wall
    m = slop of regression
    flag= 1 if left, otherwise right (-1)

    This function takes in a distance to a wall and the angle of the lidar scan from there
    It also takes in the desired distance d we want to maintain from the wall
    Output: the new distance to target point, L1 (should be offset from wall), required angle eta

    Note: +x is forward, +y is left
    """
    x_d= fwall_distance*math.cos(scan_angle) #current distance, should be positive since we are looking ahead
    y_d=fwall_distance*math.sin(scan_angle) # a little bit of trig needed to convert
    a=m 
    b=1
    if abs(a)>2: #this condition solves the driving straight into a wall
        print(a)
        if flag==1:#Left side scan [90 to 0]
            x_d-=math.sqrt(2)/2*d #sets a 45 degree angle away from the wall, to the right
            y_d-=math.sqrt(2)/2*d
        else: #right side scan
            x_d-=math.sqrt(2)/2*d #sets a 45 degree angle away from the wall, to the right
            y_d=math.sqrt(2)/2*d

    else: #just normal driving along
        if flag==1:#Left side scan [90 to 0]
            x_d+=a/(math.sqrt(a**2+b**2))*d
            y_d-=b/(math.sqrt(a**2+b**2))*d

        else: #right side scan
            x_d-=a/(math.sqrt(a**2+b**2))*d
            y_d+=b/(math.sqrt(a**2+b**2))*d


    eta=math.atan(y_d/x_d) #have to flip to get angle
    L1=math.sqrt(x_d**2+y_d**2)

    return (eta, L1)





def input_angle(eta, L, L1):
    """
    This script finds the angle that the wheel needs to achieve in order to follow the pursuit trajectory

    Args:
    eta: angle between R's
    L: Length between two points
    L1: Distance between front and back wheels.

    Returns:
    delta: required wheel angle in RADIANS!!!!
    """
    delta= math.atan(2*L/L1*math.sin(eta))

    # Return the output
    return delta