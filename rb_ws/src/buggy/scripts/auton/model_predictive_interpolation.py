from controller import Controller

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose
from std_msgs.msg import Float64, Float64MultiArray, Bool
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Pose2D

from threading import Lock

from pose import Pose
from trajectory import Trajectory
from controller import Controller
from world import World

import numpy as np


class ModelPredictiveController(Controller):
    steering_angles = None
    times = None
    headings = None
    solver_running = False
    FEEDBACK = True

    def __init__(self):
        self.lock = Lock()
        self.times = np.zeros(200) + rospy.get_time()
        self.steering_angles = np.zeros(200)
        self.headings = np.zeros(200)

        self.speed_publisher = rospy.Publisher('mpc/speed', Float64, queue_size=1)
        self.pose_publisher = rospy.Publisher('mpc/current_pose', Pose2D, queue_size=1)
        rospy.Subscriber('mpc/sol_trajectory',
                         Float64MultiArray,
                         self.update_trajectory)
        rospy.Subscriber('mpc/solver_status',
                         Bool,
                         self.update_solver_status)

    def compute_control(self, current_pose: Pose, trajectory: Trajectory, current_speed: float):
        # IF MPC SOLVER ISN'T RUNNING, SUBMIT PROBLEM
        # ELSE, DON'T
        # EITHER WAY, INTERPOLATE
        if not self.solver_running:
            self.speed_publisher.publish(current_speed)
            self.pose_publisher.publish(Pose2D(current_pose.x, current_pose.y, current_pose.theta))

        time_s = rospy.get_time()

        delta_bar = np.interp(time_s, self.times, self.steering_angles)

        delta = delta_bar
        if self.FEEDBACK:
            theta_bar = np.interp(time_s, self.times, self.headings)
            delta += 0.05 * (theta_bar - current_pose.theta)

        return delta

    def update_trajectory(self, msg):
        with self.lock:
            timed_traj = np.array(msg.data)
            timed_traj = np.reshape(timed_traj, [d.size for d in msg.layout.dim])
            self.times = timed_traj[:, 0]
            self.headings = timed_traj[:, 3]
            self.steering_angles = timed_traj[:, 4]
            self.solver_running = False

    def update_solver_status(self, msg):
        with self.lock:
            self.solver_running = True
