#! /usr/bin/env python3
import numpy as np

class My_Pose:
    def __init__(self, x, y, z, heading):
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading

class Buggy:
    WHEELBASE = 1.3
    MAX_LATERAL_ACCEL = 9.81/2 # m/s^2

class Utils:
    @staticmethod
    def get_heading_from_vec(vec: list) -> float:
        """Gets heading given a vector

        Args:
            vec (list): [x, y]

        Raises:
            Exception: If input vector is the wrong size

        Returns:
            float: degrees (heading)
        """
        if (len(vec) != 2):
            raise Exception("Vector input size != 2 (should just be x - East and y - North)")
        
        return np.arctan2(vec[0], vec[1]) # atypical bc our X is pointing East, NOT North
    
    @staticmethod
    def calculate_lateral_accel(linear_speed, steering_angle):
        radius = Buggy.WHEELBASE / np.tan(np.deg2rad(steering_angle))
        lat_accel = (linear_speed**2)/radius
        return lat_accel