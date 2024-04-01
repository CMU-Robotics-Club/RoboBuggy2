import queue

from controller import Controller

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose
from std_msgs.msg import Float64, Float64MultiArray, Bool
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Pose2D

import threading

from pose import Pose
from trajectory import Trajectory
from controller import Controller
from world import World
import numpy as np

from model_predictive_controller import ModelPredictiveController


class ModelPredictiveInterpolator(Controller):

    def __init__(self, self_name, start_index):
        super(ModelPredictiveInterpolator, self).__init__(start_index, self_name)
        self.mpc = ModelPredictiveController(self_name,
            start_index=start_index,
            ROS=False)

        # try run mpc at 100hz
        self.mpc_rate = rospy.Rate(100)
        # singleton queue
        self.problem_queue = queue.Queue(maxsize=1)

        self.t_last_solved = None

        # in seconds
        self.times = np.linspace(0,
            self.mpc.MPC_HORIZON * self.mpc.MPC_TIMESTEP,
            self.mpc.MPC_HORIZON)

        self.mpc_thread = threading.Thread(target=self.mpc_solver)
        self.mpc_thread.start()

    def mpc_solver(self):
        while not rospy.is_shutdown():
            # try to consume from q
            # blocking
            pose, traj, speed = self.problem_queue.get()
            state_trajectory = self.mpc.compute_trajectory(pose, traj, speed)
            self.solutions = state_trajectory[:, self.mpc.N_STATES - 1]
            self.t_last_solved = rospy.get_time()
            self.mpc_rate.sleep()

    def compute_control(self, current_pose: Pose, trajectory: Trajectory, current_speed: float):
        # IF MPC SOLVER ISN'T RUNNING, SUBMIT PROBLEM
        # ELSE, DON'T
        # EITHER WAY, INTERPOLATE

        # if queue empty, submit problem
        # print(self.problem_queue.empty())
        if self.problem_queue.empty():
            self.problem_queue.put((current_pose, trajectory, current_speed))

        if self.t_last_solved is None:
            return 0

        now = rospy.get_time()
        delta = np.interp(now - self.t_last_solved, self.times, self.solutions)
        print(delta - self.solutions[0])
        # print(now - self.t_last_solved)
        # print(np.arange(self.solutions.shape[0]))
        # print("sol traj idx", np.interp(now - self.t_last_solved, self.times, np.arange(self.solutions.shape[0])))

        return delta