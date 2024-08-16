import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from buggy.msg import TrajectoryMsg
from pose import Pose

from trajectory import Trajectory
from world import World

class PathPlanner():
    """
    Class to generate new trajectory splices for SC autonomous system.

    Takes in a default trajectory and an inner curb trajectory.

    """
    # move the curb towards the center of the course by CURB_MARGIN meters
    # for a margin of safety
    CURB_MARGIN = 1 #m

    # the offset is calculated as a mirrored sigmoid function of distance
    OFFSET_SCALE_CROSS_TRACK = 2 #m
    OFFSET_SCALE_ALONG_TRACK = 0.2
    ACTIVATE_OTHER_SCALE_ALONG_TRACK = 0.1
    OFFSET_SHIFT_ALONG_TRACK = 4 #m

    # number of meters ahead of the buggy to generate local trajectory for
    LOCAL_TRAJ_LEN = 50#m

    # start generating local trajectory this many meters ahead of current position
    LOOKAHEAD = 2#m

    # number of points to sample along the nominal trajectory
    RESOLUTION = 150

    def __init__(self, nominal_traj:Trajectory, left_curb:Trajectory) -> None:

        self.debug_passing_traj_publisher = rospy.Publisher(
            "/auton/debug/passing_traj", NavSatFix, queue_size=1000
        )

        self.other_buggy_xtrack_publisher = rospy.Publisher(
            "/auton/debug/other_buggy_xtrack", Float64, queue_size=1
        )

        self.traj_publisher = rospy.Publisher("SC/nav/traj", TrajectoryMsg, queue_size=1)

        self.nominal_traj = nominal_traj
        self.left_curb = left_curb

        # TODO: estimate this value based on the curvature of NAND's recent positions
        self.other_steering_angle = 0



    def offset_func(self, dist):
        """
        Args:
            dist: (N, ) numpy array, distances between ego-buggy and obstacle,
            along the trajectory
        Returns:
            (N, ) numpy array, offsets from nominal trajectory required to overtake,
            defined by a sigmoid function
        """

        return self.OFFSET_SCALE_CROSS_TRACK / \
            (1 + np.exp(-(-self.OFFSET_SCALE_ALONG_TRACK * dist +
            self.OFFSET_SHIFT_ALONG_TRACK)))

    def activate_other_crosstrack_func(self, dist):
        """
        Args:
            dist: (N, ) numpy array, distances between ego-buggy and obstacle,
            along the trajectory
        Returns:
            (N, ) numpy array, multiplier used to weigh the cross-track distance of
            the obstacle into the passing offset calculation.
        """
        return 1 / \
            (1 + np.exp(-(-self.ACTIVATE_OTHER_SCALE_ALONG_TRACK * dist +
            self.OFFSET_SHIFT_ALONG_TRACK)))


    def compute_traj(
        self,
        self_pose: Pose,
        other_pose: Pose, #Currently NAND's location -- To be Changed
        ) -> Trajectory:
        """
        draw trajectory starting at the current pose and ending at a fixed distance
        ahead. For each trajectory point, calculate the required offset perpendicular to the nominal
        trajectory. A sigmoid function of the distance along track to the other buggy is used to
        weigh the other buggy's cross-track distance. This calculation produces a line that
        allows the ego-buggy's trajectory to go through the other buggy. Since we want to pass
        the other buggy at some constant distance to the left, another sigmoid function is multiplied
        by that constant distance to produce a smooth trajectory that passes the other buggy.

        Finally, the trajectory is bounded to the left by the left curb (if it exists), and to the right
        by the nominal trajectory. (we never pass on the right)

        passing offsets =
            activate_other_crosstrack_func(distance to other buggy along track) *
            other buggy cross track distance +
            offset_func(distance to other buggy along track)

        trajectory = nominal trajectory +
            left nominal trajectory unit normal vector *
            clamp(passing offsets, 0, distance from nominal trajectory to left curb)

        Args:
            other_pose (Pose): Pose containing NAND's easting (x),
                northing(y), and heading (theta), in "world" cooridnates,
                which is UTM, shifted so that the origin is near the course.

                See world.py

            other_speed (float): speed in m/s of NAND

        Returns:
            Trajectory: new trajectory object for ego-buggy to follow.
        """
        # grab slice of nominal trajectory
        nominal_idx = self.nominal_traj.get_closest_index_on_path(self_pose.x, self_pose.y)
        nominal_dist_along = self.nominal_traj.get_distance_from_index(nominal_idx)

        nominal_slice = np.empty((self.RESOLUTION, 2))

        # compute the distance along nominal trajectory between samples and the obstacle
        nominal_slice_dists = np.linspace(
            nominal_dist_along + self.LOOKAHEAD,
            nominal_dist_along + self.LOOKAHEAD + self.LOCAL_TRAJ_LEN,
            self.RESOLUTION)

        for i in range(self.RESOLUTION):
            nominal_slice[i, :] = np.array(self.nominal_traj.get_position_by_distance(
               nominal_slice_dists[i]
            ))

        # get index of the other buggy along the trajetory and convert to distance
        other_idx = self.nominal_traj.get_closest_index_on_path(other_pose.x, other_pose.y)
        other_dist = self.nominal_traj.get_distance_from_index(other_idx)
        nominal_slice_to_other_dist = np.abs(nominal_slice_dists - other_dist)

        passing_offsets = self.offset_func(nominal_slice_to_other_dist)

        # compute signed cross-track distance between NAND and nominal
        nominal_to_other = np.array((other_pose.x, other_pose.y)) - \
            np.array(self.nominal_traj.get_position_by_index(other_idx))

        # dot product with the unit normal to produce left-positive signed distance
        other_normal = self.nominal_traj.get_unit_normal_by_index(np.array(other_idx.ravel()))
        other_cross_track_dist = np.sum(
            nominal_to_other * other_normal, axis=1)

        self.other_buggy_xtrack_publisher.publish(Float64(other_cross_track_dist))

        # here, use passing offsets to weight NAND's cross track signed distance:
        # if the sample point is far from SC, the cross track distance doesn't matter
        # if the sample point is near, we add cross track distance to the offset,
        # such that the resulting offset is adjusted by position of NAND

        passing_offsets = passing_offsets + \
            self.activate_other_crosstrack_func(nominal_slice_to_other_dist) * other_cross_track_dist

        # clamp passing offset distances to distance to the curb
        if self.left_curb is not None:
            # grab slice of curb correponding to slice of nominal trajectory.
            curb_idx = self.left_curb.get_closest_index_on_path(self_pose.x, self_pose.y)
            curb_dist_along = self.left_curb.get_distance_from_index(curb_idx)
            curb_idx_end = self.left_curb.get_closest_index_on_path(nominal_slice[-1, 0], nominal_slice[-1, 1])
            curb_dist_along_end = self.left_curb.get_distance_from_index(curb_idx_end)
            curb_dists = np.linspace(curb_dist_along, curb_dist_along_end, self.RESOLUTION)

            curb_slice = np.empty((self.RESOLUTION, 2))
            for i in range(self.RESOLUTION):
                curb_slice[i, :] = np.array(self.left_curb.get_position_by_distance(
                    curb_dists[i]
                ))

            # compute distances from the sample points to the curb
            nominal_slice_to_curb_dist = np.linalg.norm(curb_slice - nominal_slice, axis=1)
            passing_offsets = np.minimum(passing_offsets, nominal_slice_to_curb_dist - self.CURB_MARGIN)

        # clamp negative passing offsets to zero, since we always pass on the left,
        # the passing offsets should never pull SC to the right.
        passing_offsets = np.maximum(0, passing_offsets)

        # shift the nominal slice by passing offsets
        nominal_normals = self.nominal_traj.get_unit_normal_by_index(
            self.nominal_traj.get_index_from_distance(nominal_slice_dists)
        )
        positions = nominal_slice + (passing_offsets[:, None] * nominal_normals)

        # prepend current pose
        # positions = np.vstack((np.array([self_pose.x, self_pose.y]), positions))

        # publish passing targets for debugging
        for i in range(len(positions)):
            reference_navsat = NavSatFix()
            ref_gps = World.world_to_gps(positions[i, 0], positions[i, 1])
            reference_navsat.latitude = ref_gps[0]
            reference_navsat.longitude = ref_gps[1]
            self.debug_passing_traj_publisher.publish(reference_navsat)

        local_traj = Trajectory(json_filepath=None, positions=positions)
        self.traj_publisher.publish(local_traj.pack(self_pose.x, self_pose.y))
        return local_traj, \
                local_traj.get_closest_index_on_path(
                self_pose.x,
                self_pose.y)