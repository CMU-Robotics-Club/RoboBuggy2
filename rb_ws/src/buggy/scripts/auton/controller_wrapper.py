import time
from threading import Lock

from numba import njit

import numpy as np
import osqp
from scipy import sparse
import copy

import rospy
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import Pose2D

from pose import Pose
from trajectory import Trajectory
from controller import Controller
from world import World


import matplotlib.pyplot as plt

from stanley_controller import StanleyController
from model_predictive_controller import ModelPredictiveController

class ControllerWrapper(Controller):
    """
    Controller Wrapper that combines MPC and Stanley
    """

    DEBUG = True
    PLOT = False
    TIME = False
    ROS = True

    def __init__(self, buggy_name, start_index=0) -> None:
        super(ControllerWrapper, self).__init__(start_index, buggy_name)
        self.buggy_name = buggy_name
        self.start_index = start_index

        self.MPC_controller = ModelPredictiveController(buggy_name, start_index)
        self.stanley_controller = StanleyController(buggy_name, start_index)

    def compute_control(self, current_pose: Pose, trajectory: Trajectory, current_speed: float):
        traj = copy.deepcopy(trajectory)
        print(current_pose, trajectory, current_speed)
        stanley_steer, stanley_solved = self.stanley_controller.compute_control(current_pose, traj, current_speed)
        print(current_pose, trajectory, current_speed)
        mpc_steer, mpc_solved = 0, False #self.MPC_controller.compute_control(current_pose, traj, current_speed)
        print(current_pose, trajectory, current_speed)
        self.steer = mpc_steer
        if not mpc_solved:
            print("SWITCHING TO STANLEY")
            self.steer = stanley_steer
        print(mpc_steer, stanley_steer, self.steer)
        print("\n")
        return self.steer, mpc_solved or stanley_solved 
        