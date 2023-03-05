from abc import ABC, abstractmethod

import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose

from pose import Pose
from trajectory import Trajectory
from controller import Controller
from world import World


class PurePursuitController(Controller):
    """
    Pure Pursuit Controller
    """

    WHEELBASE = 1.3

    LOOK_AHEAD_DIST_CONST = 0.375
    MIN_LOOK_AHEAD_DIST = 0.5
    MAX_LOOK_AHEAD_DIST = 5

    current_traj_index = 0

    def __init__(self) -> None:
        self.debug_reference_pos_publisher = rospy.Publisher(
            "auton/debug/reference_navsat", NavSatFix, queue_size=1
        )
        self.debug_track_pos_publisher = rospy.Publisher(
            "auton/debug/track_navsat", NavSatFix, queue_size=1
        )
        self.debug_error_publisher = rospy.Publisher(
            "auton/debug/error", ROSPose, queue_size=1
        )

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
        if self.current_traj_index >= trajectory.get_num_points() - 1:
            return 0

        traj_index = trajectory.get_closest_index_on_path(
            current_pose.x,
            current_pose.y,
            start_index=self.current_traj_index,
            end_index=self.current_traj_index + 10,
        )
        self.current_traj_index = max(traj_index, self.current_traj_index)
        traj_dist = trajectory.get_distance_from_index(traj_index)

        reference_position = trajectory.get_position_by_index(traj_index)
        reference_error = current_pose.convert_point_from_global_to_local_frame(
            reference_position
        )

        lookahead_dist = np.clip(
            self.LOOK_AHEAD_DIST_CONST * current_speed,
            self.MIN_LOOK_AHEAD_DIST,
            self.MAX_LOOK_AHEAD_DIST,
        )
        traj_dist += lookahead_dist

        track_position = trajectory.get_position_by_distance(traj_dist)
        track_error = current_pose.convert_point_from_global_to_local_frame(
            track_position
        )

        bearing = np.arctan2(track_error[1], track_error[0])
        steering_angle = np.arctan(
            2.0 * self.WHEELBASE * np.sin(bearing) / lookahead_dist
        )

        # Publish track position for debugging
        track_navsat = NavSatFix()
        track_gps = World.world_to_gps(*track_position)
        track_navsat.latitude = track_gps[0]
        track_navsat.longitude = track_gps[1]
        self.debug_track_pos_publisher.publish(track_navsat)

        # Publish reference position for debugging
        reference_navsat = NavSatFix()
        ref_pos = trajectory.get_position_by_distance(traj_dist - lookahead_dist)
        ref_gps = World.world_to_gps(*ref_pos)
        reference_navsat.latitude = ref_gps[0]
        reference_navsat.longitude = ref_gps[1]
        self.debug_reference_pos_publisher.publish(reference_navsat)

        # Publish error for debugging
        error_pose = ROSPose()
        error_pose.position.x = reference_error[0]
        error_pose.position.y = reference_error[1]
        self.debug_error_publisher.publish(error_pose)

        return steering_angle
