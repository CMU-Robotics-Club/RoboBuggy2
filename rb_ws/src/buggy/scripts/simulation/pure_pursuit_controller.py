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
import time

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

    def get_next_coord(self, pose, look_ahead_dist, look_ahead_angle_deg) -> My_Pose:
        """Get the next coordinates the buggy needs to look at.

        Args:
            pose (Pose): pose in ROS format, ENU frame
            look_ahead_dist (float): distance in meters
            look_ahead_angle_deg (float): angle range in which we should look ahead (each side)
        
        Returns:
            My_Pose: custom pose object of the lookahead coordinates
        """
        # get heading from ENU position
        # get 
        q_x = pose.orientation.x
        q_y = pose.orientation.y
        q_z = pose.orientation.z
        q_w = pose.orientation.w
        (_, _, heading_deg) = np.rad2deg(euler_from_quaternion([q_x, q_y, q_z, q_w]))

        # convert heading to north clockwise (compass)
        if (heading_deg >= -90):
            heading_deg = 90 - heading_deg 
        else:
            heading_deg = -270 - heading_deg

        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        next_coord = self.path.iloc[self.curr_path_idx]
        next_x = next_coord["x"]
        next_y = next_coord["y"]
        next_z = next_coord["z"]
        delta_x = next_x - x
        delta_y = next_y - y
        delta_z = next_z - z

        heading_to_next_pathpoint = Utils.get_heading_from_vec_deg([delta_x, delta_y])
        delta_heading_deg = np.abs(next_coord["heading"] - heading_deg)

        # ignore z in dist calculations
        dist = (delta_x**2 + delta_y**2)**0.5

        # Loop until found appropriate next pose that satisfies look_ahead_angle and
        # look_ahead_dist
        while (dist <= look_ahead_dist and delta_heading_deg <= look_ahead_angle_deg):
            self.curr_path_idx += 1
            next_coord = self.path.iloc[self.curr_path_idx]
            next_x = next_coord["x"]
            next_y = next_coord["y"]
            next_z = next_coord["z"]

            delta_x = next_x - x

            delta_y = next_y - y
            delta_z = next_z - z

            heading_to_next_pathpoint = Utils.get_heading_from_vec_deg([delta_x, delta_y])
            delta_heading_deg = heading_to_next_pathpoint - heading_deg

            dist = (delta_x**2 + delta_y**2)**0.5

        print("dist: ", dist)
        print("LH_dist: ", look_ahead_dist)
        print("cur heading:", heading_deg)
        print("next heading:",  next_coord["heading"])
        print("heading to go to next target: ", heading_to_next_pathpoint)
        print("delta heading: ", delta_heading_deg)
        print("LH angle: ", look_ahead_angle_deg)
        print("cur_path_idx, ", self.curr_path_idx)
        print("this coord:", pose)
        print("next coord:", next_coord)
            
        return My_Pose(next_x, next_y, next_z, next_coord["heading"])
    
    def calculate_steering_angle_deg(self, current_pose: My_Pose, next_pose: My_Pose) -> float:
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

        alpha_rad = np.deg2rad(Utils.get_heading_from_vec_deg([delta_x, delta_y]))

        l_d = (delta_x**2 + delta_y**2 + delta_z**2)**0.5

        L_WB = Buggy.WHEELBASE

        return np.rad2deg(np.arctan(2.0*L_WB*np.sin(alpha_rad)/l_d))
    
    @staticmethod
    def calculate_look_ahead_dist(speed):
        dist = speed * Pure_Pursuit.LOOK_AHEAD_DIST_CONST
        return np.clip(dist, Pure_Pursuit.MIN_LOOK_AHEAD_DIST, Pure_Pursuit.MAX_LOOK_AHEAD_DIST)
    
    def step(self):
        """Preliminary control loop. Call this at a set frequency
        """
        print("pure pursuit step")
        with self.lock:
            speed = self.speed        
            steering_angle_deg = self.steering_angle_deg
            pose = self.pose

        # only calculate lateral accel for nonzero steering angle
        if (steering_angle_deg == 0 or Utils.calculate_lateral_accel(speed, steering_angle_deg) > Buggy.MAX_LATERAL_ACCEL):
            self.cmd_braking(True)
        else:
            self.cmd_braking(False)
        
        next_pose = self.get_next_coord(pose, Pure_Pursuit.calculate_look_ahead_dist(speed), 45.0)
        
        q_x = pose.orientation.x
        q_y = pose.orientation.y
        q_z = pose.orientation.z
        q_w = pose.orientation.w
        (_, _, heading_deg) = np.rad2deg(euler_from_quaternion([q_x, q_y, q_z, q_w]))

        x = pose.position.x
        y = pose.position.y
        z = pose.position.z # NOTE: Check NED frame of INS, what's the Units for DOWN?

        current_pose = My_Pose(x, y, z, heading_deg)

        steering_cmd = self.calculate_steering_angle_deg(current_pose, next_pose)
        print("steering cmd:", steering_cmd)
        self.cmd_steering(steering_cmd)
    
    def run(self):
        rate = rospy.Rate(self.RATE)
        time.sleep(2)
        while not rospy.is_shutdown():
            print("HELLO")
            rate.sleep()
            self.step()


if __name__ == '__main__':
  rospy.init_node("pure_pursuit_controller")
  controller = Pure_Pursuit("/rb_ws/src/buggy/paths/out.csv")
  controller.run()
