from abc import ABC, abstractmethod

import numpy as np
from trajectory import Trajectory
from pose import Pose
import rospy
from sensor_msgs.msg import NavSatFix
from world import World


class Controller(ABC):
    """
    Base class for all controllers.

    The controller takes in the current state of the buggy and a reference
    trajectory. It must then compute the desired control output.

    The method that it does this by is up to the implementation, of course.
    Example schemes include Pure Pursuit, Stanley, and LQR.
    """

    NAND_WHEELBASE = 1.3
    SC_WHEELBASE = 1.104
    WHEELBASE = SC_WHEELBASE
    current_traj_index = 0

    def __init__(self, start_index, buggy_name) -> None:
        self.buggy_name = buggy_name
        # self.trajectory_forward_1 = rospy.Publisher(
        #     buggy_name + "/auton/debug/forward1_navsat", NavSatFix, queue_size=1
        # )
        # self.trajectory_forward_2 = rospy.Publisher(
        #     buggy_name + "/auton/debug/forward2_navsat", NavSatFix, queue_size=1
        # )
        # self.trajectory_forward_3 = rospy.Publisher(
        #     buggy_name + "/auton/debug/forward3_navsat", NavSatFix, queue_size=1
        # )
        # self.trajectory_backward_1 = rospy.Publisher(
        #     buggy_name + "/auton/debug/backward1_navsat", NavSatFix, queue_size=1
        # )
        # # Make lists of publishers for easy iteration
        # self.forward_publishers = [self.trajectory_forward_1, self.trajectory_forward_2, self.trajectory_forward_3]
        # self.backward_publishers = [self.trajectory_backward_1]
        self.current_traj_index = start_index

    @abstractmethod
    def compute_control(
        self, current_pose: Pose, trajectory: Trajectory, current_speed: float
    ):
        """
        Computes the desired control output given the current state and reference trajectory

        Args:
            state (numpy.ndarray [size: (3,)]): current pose (x, y, theta)
            trajectory (Trajectory): reference trajectory
            current_speed (float): current speed of the buggy

        Returns:
            float (desired steering angle)
        """
        raise NotImplementedError
    
    def plot_trajectory(
        self, current_pose: Pose, trajectory: Trajectory, current_speed: float
    ):
        """
        Plots the next x and previous y points along the trajectory

        Args:
            current_pose (Pose): current pose (x, y, theta) (UTM coordinates)
            trajectory (Trajectory): reference trajectory
            current_speed (float): current speed of the buggy
        """

        # Compute distance along path from current position
        traj_dist = trajectory.get_distance_from_index(self.current_traj_index)

        # Plot forward projections
        for i in range(1, len(self.forward_publishers)+1):
            forward_position = trajectory.get_position_by_distance(traj_dist + i*current_speed)
            navsat = NavSatFix()
            forward_gps = World.world_to_gps(*forward_position)
            navsat.latitude = forward_gps[0]
            navsat.longitude = forward_gps[1]
            self.forward_publishers[i-1].publish(navsat)

        # Plot backward positions
        for i in range(1, len(self.backward_publishers)+1):
            backward_position = trajectory.get_position_by_distance(traj_dist - i*current_speed)
            navsat = NavSatFix()
            backward_gps = World.world_to_gps(*backward_position)
            navsat.latitude = backward_gps[0]
            navsat.longitude = backward_gps[1]
            self.backward_publishers[i-1].publish(navsat)





