import numpy as np
from pose import Pose

class Projector:
    """
    Class for buggy motion projection
    """
    def __init__(self, wheelbase: float):
        self.wheelbase = wheelbase

    
    def project(self, pose: Pose, command: float, v: float, time: float, resolution: int) -> list:
        """
        Project buggy motion analytically. Assumes constant velocity and turning angle for the duration. 

        Args:
            pose (Pose): Pose containing utm_e, utm_n, and heading, in UTM coords and degrees
            command (float): Turning angle, in degrees
            v (float): Buggy velocity, in m/s
            time (float): Time to look ahead, in s
            resolution (int): Number of points to output per second

        Returns:
            list: List containing 3-tuples of utm_e, utm_n, heading positions along the projected arc.
        """
        dtheta = v * np.tan(np.deg2rad(command)) / self.wheelbase
        ts = 1/resolution
        t = 0
        output = []
        
        for i in range(int(resolution * time)):
            t += ts
            theta = t * dtheta + np.deg2rad(pose.theta)
            if dtheta != 0:
                x = pose.x + (v / dtheta) * np.sin(theta)
                y = pose.y + (v / dtheta) * (1-np.cos(theta))
            else:
                x = pose.x + t * v * np.cos(theta)
                y = pose.y + t * v * np.sin(theta)

            output.append((x, y, np.rad2deg(theta)))
        
        return output

    def project_discrete(self, pose: Pose, command: float, v: float, time: float, resolution: int, sim_rate: int) -> list:
        """
        Project buggy motion discretely, performing kinematics at each sim_ts. Assumes constant velocity and turning angle for the duration. 

        Args:
            pose (Pose): Pose containing utm_e, utm_n, and heading, in UTM coords and degrees
            command (float): Turning angle, in degrees
            v (float): Buggy velocity, in m/s
            time (float): Time to look ahead, in s
            resolution (int): Number of points to output per second
            sim_rate (int): Number of simulation timesteps per second

        Returns:
            list: List containing 3-tuples of utm_e, utm_n, heading positions along the projected arc.
        """
        ts = 1/resolution
        sim_ts = 1/sim_rate
        t = 0
        output = []
        x = pose.x
        y = pose.y
        theta = np.deg2rad(pose.theta)
        comm = np.deg2rad(command)
        next_out = ts

        while t < time:
            x += sim_ts * v * np.cos(theta)
            y += sim_ts * v * np.sin(theta)
            theta += sim_ts * (v/self.wheelbase) * np.tan(comm)

            t += sim_ts
            if t >= next_out:
                output.append((x, y, np.rad2deg(theta)))
                next_out += ts
        
        return output