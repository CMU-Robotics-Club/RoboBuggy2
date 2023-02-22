#! /usr/bin/env python3
import rospy

# ROS Message Imports
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Pose


import threading
import numpy as np
import pandas as pd
from tf.transformations import euler_from_quaternion
from utils import *

from controller import Controller

class Pure_Pursuit(Controller):
    """Inherits from Controller. Implements Pure Pursuit controller where the look-ahead distance
    varies can vary based on buggy current speed, desried speed, current error, etc.
    """
    
    LOOK_AHEAD_DIST_CONST = 0.5
    MIN_LOOK_AHEAD_DIST = 4.0
    MAX_LOOK_AHEAD_DIST = 15.0

    def __init__(self, path_file):
        """
        Args:
            path_file (str): Path to the csv file to load the waypoints for GPS coordinates
        """
        Controller.__init__(self)
        self.path = pd.read_csv(path_file)
        print("Loaded Path File: " + str(path_file))
        
        self.curr_path_idx = 0 # idx we're currently looking at in the path file

    def get_next_coord(self, pose, look_ahead_dist, look_ahead_angle) -> My_Pose:
        """Get the next coordinates the buggy needs to look at.

        Args:
            pose (Pose): pose in ROS format
            look_ahead_dist (float): distance in meters
            look_ahead_angle (float): angle range in which we should look ahead (each side)
        
        Returns:
            My_Pose: custom pose object of the lookahead coordinates
        """
        
        q_x = pose.orientation.x
        q_y = pose.orientation.y
        q_z = pose.orientation.z
        q_w = pose.orientation.w
        (_, _, heading) = np.rad2deg(euler_from_quaternion([q_x, q_y, q_z, q_w]))

        x = pose.x
        y = pose.y
        z = pose.z # NOTE: Check NED frame of INS, what's the Units for DOWN?
        
        next_coord = self.path.iloc[self.curr_path_idx]
        next_x = next_coord["x"]
        next_y = next_coord["y"]
        next_z = next_coord["z"]
        delta_x = next_x - x
        delta_y = next_y - y
        delta_z = next_z - z

        delta_heading = Utils.get_angle_from_vec([delta_x, delta_y])

        dist = ((next_x - x)**2 + (next_y - y)**2 + (next_z - z)**2)**0.5

        # Loop until found appropriate next pose that satisfies look_ahead_angle and
        # look_ahead_dist
        while (dist <= look_ahead_dist and delta_heading <= look_ahead_angle):
            self.curr_path_idx += 1
            next_coord = self.path.iloc[self.curr_path_idx]
            next_x = next_coord["x"]
            next_y = next_coord["y"]
            next_z = next_coord["z"]

            delta_x = next_x - x
            delta_y = next_y - y
            delta_z = next_z - z

            delta_heading = Utils.get_angle_from_vec([delta_x, delta_y])

            dist = (delta_x**2 + delta_y**2 + delta_z**2)**0.5
            
        return My_Pose(next_x, next_y, next_z, next_coord["heading"])
    
    def calculate_steering_angle(self, current_pose: My_Pose, next_pose: My_Pose) -> float:
        """Calculate the steering angle for the buggy given our current position and where we
        would like to go next.

        Args:
            current_pose (My_Pose): _description_
            next_pose (My_Pose): _description_

        Returns:
            float: _description_
        """
        x = current_pose.x
        y = current_pose.y
        z = current_pose.z

        next_x = next_pose.x
        next_y = next_pose.y
        next_z = next_pose.z

        delta_x = next_x - x
        delta_y = next_y - y
        delta_z = next_z - z

        # Following https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf

        alpha = Pure_Pursuit.get_heading_from_vec([delta_x, delta_y])

        l_d = (delta_x**2 + delta_y**2 + delta_z**2)**0.5

        L_WB = Buggy.WHEELBASE

        return np.arctan(2.0*L_WB*np.sin(alpha)/l_d)
    
    @staticmethod
    def calculate_look_ahead_dist(speed):
        dist = speed * Pure_Pursuit.LOOK_AHEAD_DIST_CONST
        return np.clip(dist, Pure_Pursuit.MIN_LOOK_AHEAD_DIST, Pure_Pursuit.MAX_LOOK_AHEAD_DIST)
    
    def step(self):
        """Preliminary control loop. Call this at a set frequency
        """
        with self.lock:
            speed = self.speed
            steering_angle = self.steering_angle
            pose = self.pose
        if (Utils.calculate_lateral_accel(speed, steering_angle) > Buggy.MAX_LATERAL_ACCEL):
            self.cmd_braking(True)
        else:
            self.cmd_braking(False)
        
        next_pose = self.get_next_coord(pose, Pure_Pursuit.calculate_look_ahead_dist(speed), 45.0)
        
        q_x = pose.orientation.x
        q_y = pose.orientation.y
        q_z = pose.orientation.z
        q_w = pose.orientation.w
        (_, _, heading) = np.rad2deg(euler_from_quaternion([q_x, q_y, q_z, q_w]))

        x = pose.x
        y = pose.y
        z = pose.z # NOTE: Check NED frame of INS, what's the Units for DOWN?

        current_pose = My_Pose(x, y, z, heading)

        steering_cmd = self.calculate_steering_angle(current_pose, next_pose)

        self.cmd_steering(steering_cmd)


if __name__ == '__main__':
  rospy.init_node("pure_pursuit_controller")
  controller = Pure_Pursuit("/rb_ws/src/buggy/paths/run1.csv")
  controller.run()
