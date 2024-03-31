import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from pose import Pose

from occupancy_grid.grid_manager import OccupancyGrid
from path_projection import Projector
from trajectory import Trajectory
from world import World
class PathPlanner():
    LOOKAHEAD_TIME = 2.0 #s
    RESOLUTION = 30 #samples/s
    PASSING_OFFSET = 2 #m
    # in meters, the number of meters behind NAND before we start morphing the trajectory
    REAR_MARGIN = 10 #m

    # in meters, the number of meters in front of NAND,
    # before the morphed trajectory rejoins the nominal trajectory
    # WARNING: set this value to be greater than 15m/s * lookahead time (10 m/s is the upper limit
    # of NAND speed) Failure to do so can result violent u-turns in the new trajectory.
    FRONT_MARGIN = 35 #m

    def __init__(self, nominal_traj:Trajectory) -> None:
        self.occupancy_grid = OccupancyGrid()

        # TODO: update with NAND wheelbase
        self.path_projector = Projector(1.3)

        self.debug_passing_traj_publisher = rospy.Publisher(
            "/auton/debug/passing_traj", NavSatFix, queue_size=1000
        )

        self.debug_splice_pt_publisher = rospy.Publisher(
            "/auton/debug/splice_pts", NavSatFix, queue_size=1000
        )


        self.debug_grid_cost_publisher = rospy.Publisher(
            "/auton/debug/grid_cost", Float64, queue_size=0
        )
        self.nominal_traj = nominal_traj

        # TODO: estimate this value based on the curvature of NAND's recent positions
        self.other_steering_angle = 0

    def compute_traj(
        self,
        other_pose: Pose, #Currently NAND's location -- To be Changed
        other_speed: float) -> Trajectory:
        """
        draw trajectory, such that the section of the
        trajectory near NAND is replaced by a new segment:
        #1. the nominal trajectory, until REAR_MARGIN behind NAND
        #2. passing targets
        #3. nominal trajectory, starting at FRONT_MARGIN ahead of NAND

        To generate the passing targets, we take NAND's future positions,
        shifted along normal of nominal trajectory by PASSING_OFFSET

        Since they are shifted by a fixed distance, this approach assumes NAND's
        trajectory is similar to that of ego-buggy (short-circuit).

        TODO: shift the future positions while taking into account smoothness of the path,
        as well as curbs, such that even if NAND's trajectory is different from ego-buggy,
        the generated passing targets will be safe.

        Args:
            other_pose (Pose): Pose containing NAND's easting (x),
                northing(y), and heading (theta), in "world" cooridnates,
                which is UTM, shifted so that the origin is near the course.

                See world.py

            other_speed (float): speed in m/s of NAND

        Returns:
            Trajectory: new trajectory object for ego-buggy to follow.
        """

        other_idx:float = self.nominal_traj.get_closest_index_on_path(
            other_pose.x,
            other_pose.y)
        #other is just NAND, for general purposes consider it other

        new_segment_start_idx:float = self.nominal_traj.get_index_from_distance(
                self.nominal_traj.get_distance_from_index(other_idx) - self.REAR_MARGIN
            ) #Where new path index starts, CONSTANT delta from NAND

        new_segment_end_idx:float = self.nominal_traj.get_index_from_distance(
                self.nominal_traj.get_distance_from_index(other_idx) + self.FRONT_MARGIN
            )

        # project other buggy path
        # TODO: put other buggy command
        other_future_poses:list = self.path_projector.project(
            other_pose,
            self.other_steering_angle,
            other_speed,
            self.LOOKAHEAD_TIME,
            self.RESOLUTION)

        other_future_poses_idxs = np.empty((len(other_future_poses), ))

        # TODO: optimize this lookup -- how tho
        for i in range(len(other_future_poses)):
            other_future_poses_idxs[i] = self.nominal_traj.get_closest_index_on_path(
                    other_future_poses[i][0],
                    other_future_poses[i][1],
                    start_index=other_idx)

        future_pose_unit_normal:np.typing.NDArray = self.nominal_traj.get_unit_normal_by_index(
            other_future_poses_idxs
        )
        passing_targets = other_future_poses + self.PASSING_OFFSET * future_pose_unit_normal

        pre_slice = self.nominal_traj.positions[:int(new_segment_start_idx), :]
        post_slice = self.nominal_traj.positions[int(new_segment_end_idx):, :]
        new_path = np.vstack((pre_slice, passing_targets, post_slice))

        # publish passing targets for debugging
        for i in range(len(passing_targets)):
            reference_navsat = NavSatFix()
            ref_gps = World.world_to_gps(passing_targets[i, 0], passing_targets[i, 1])
            reference_navsat.latitude = ref_gps[0]
            reference_navsat.longitude = ref_gps[1]
            self.debug_passing_traj_publisher.publish(reference_navsat)

        # for debugging:
        # publish the first and last point of the part of the original trajectory
        # that got spliced out
        reference_navsat = NavSatFix()
        ref_gps = World.world_to_gps(*self.nominal_traj.get_position_by_index(int(new_segment_start_idx)))
        reference_navsat.latitude = ref_gps[0]
        reference_navsat.longitude = ref_gps[1]
        self.debug_splice_pt_publisher.publish(reference_navsat)

        ref_gps = World.world_to_gps(*self.nominal_traj.get_position_by_index(int(new_segment_end_idx)))
        reference_navsat.latitude = ref_gps[0]
        reference_navsat.longitude = ref_gps[1]
        self.debug_splice_pt_publisher.publish(reference_navsat)


        # generate new path
        return Trajectory(json_filepath=None, positions=new_path)