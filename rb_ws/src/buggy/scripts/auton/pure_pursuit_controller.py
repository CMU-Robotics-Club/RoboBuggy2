from abc import ABC, abstractmethod

import numpy as np

from pose import Pose
from trajectory import Trajectory
from controller import Controller


class PurePursuitController(Controller):
    """
    Pure Pursuit Controller
    """

    WHEELBASE = 1.3

    LOOK_AHEAD_DIST_CONST = 0.5
    MIN_LOOK_AHEAD_DIST = 4.0
    MAX_LOOK_AHEAD_DIST = 15.0

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

        traj_index = trajectory.get_closest_index_on_path(
            current_pose.x, current_pose.y
        )
        traj_dist = trajectory.get_distance_from_index(traj_index)

        lookahead_dist = np.clip(
            self.LOOK_AHEAD_DIST_CONST * current_speed,
            self.MIN_LOOK_AHEAD_DIST,
            self.MAX_LOOK_AHEAD_DIST,
        )
        traj_dist += lookahead_dist

        reference_position = trajectory.get_position_by_distance(traj_dist)
        error = current_pose.convert_point_from_global_to_local_frame(
            reference_position
        )

        bearing = np.arctan2(error[1], error[0])

        steering_angle = np.arctan(
            2.0 * self.WHEELBASE * np.sin(bearing) / lookahead_dist
        )

        return steering_angle
