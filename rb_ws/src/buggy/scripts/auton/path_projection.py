import numpy as np
#from pose import Pose

class Pose:
    """
    A data structure for storing 2D poses, as well as a set of
    convenience methods for transforming/manipulating poses

    """
    x = None
    y = None
    theta = None
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

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
            pose (Pose): Pose containing x, y, and heading, in m, m, and rad
            command (float): Turning angle, in rad
            v (float): Buggy velocity, in m/s
            time (float): Time to look ahead, in s
            resolution (int): Number of points to output per second

        Returns:
            list: List containing 3-tuples of x, y, theta positions along the projected arc.
        """
        dtheta = v * np.tan(command) / self.wheelbase
        ts = 1/resolution
        t = 0
        output = []
        
        for i in range(int(resolution * time)):
            t += ts
            theta = t * dtheta + pose.theta
            x = pose.x + (v / dtheta) * np.sin(theta)
            y = pose.y + (v / dtheta) * (1-np.cos(theta))

            output.append((x, y, theta))
        
        return output

    def project_discrete(self, pose: Pose, command: float, v: float, time: float, resolution: int, sim_ts: float) -> list:
        """
        Project buggy motion discretely, performing kinematics at each sim_ts. Assumes constant velocity and turning angle for the duration. 

        Args:
            pose (Pose): Pose containing x, y, and heading, in m, m, and rad
            command (float): Turning angle, in rad
            v (float): Buggy velocity, in m/s
            time (float): Time to look ahead, in s
            resolution (int): Number of points to output per second
            sim_ts (float): Performs a timestep of kinematics for every sim_ts seconds.

        Returns:
            list: List containing 3-tuples of x, y, theta positions along the projected arc.
        """
        ts = 1/resolution
        t = 0
        output = []
        x = pose.x
        y = pose.y
        theta = pose.theta
        next_out = ts

        while t < time:
            x += sim_ts * v * np.cos(theta)
            y += sim_ts * v * np.sin(theta)
            theta += sim_ts * (v/self.wheelbase) * np.tan(command)

            t += sim_ts
            if t >= next_out:
                output.append((x, y, theta))
                next_out += ts
        
        return output