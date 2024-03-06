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
    RESOLUTION = 10 #samples/s

    # move the curb towards the center of the course by CURB_MARGIN meters
    # for a margin of safety
    CURB_MARGIN = 1 #m
    # the number of meters behind NAND before we start morphing the trajectory
    REAR_MARGIN = 10 #m

    # in meters, the number of meters in front of NAND,
    # before the morphed trajectory rejoins the nominal trajectory
    # WARNING: set this value to be greater than 15m/s * lookahead time (15 m/s is the upper limit
    # of NAND speed) Failure to do so can result violent u-turns in the new trajectory.
    FRONT_MARGIN = 35 #m

    def __init__(self, nominal_traj:Trajectory, left_curb:Trajectory) -> None:
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
        self.left_curb = left_curb

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

        left_curb_idx = self.left_curb.get_closest_index_on_path(
            other_pose.x,
            other_pose.y)

        left_curb_end_idx = self.left_curb.get_index_from_distance(
                self.nominal_traj.get_distance_from_index(left_curb_idx) + self.FRONT_MARGIN
            )

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

        other_poses_idx_along_nominal = np.empty((len(other_future_poses), ))

        # indexes of points along the left curb that are closest NAND's future poses
        left_curb_idxes = np.empty((len(other_future_poses), ))

        # TODO: optimize this lookup -- how tho
        for i in range(len(other_future_poses)):
            other_poses_idx_along_nominal[i] = self.nominal_traj.get_closest_index_on_path(
                    other_future_poses[i][0],
                    other_future_poses[i][1],
                    start_index=other_idx)

            left_curb_idxes[i] = self.left_curb.get_closest_index_on_path(
                    other_future_poses[i][0],
                    other_future_poses[i][1],
                    start_index=left_curb_idx,
                    end_index=left_curb_end_idx)

        nominal_traj_unit_normal:np.typing.NDArray = self.nominal_traj.get_unit_normal_by_index(
            other_poses_idx_along_nominal
        )

        nominal_traj_slice = self.nominal_traj.get_position_by_index(
            other_poses_idx_along_nominal
        )


        left_curb_unit_normal = self.left_curb.get_unit_normal_by_index(left_curb_idxes)

        # grab points on the left curb that run next to future trajectory of NAND
        left_curb_slice = self.left_curb.get_position_by_index(left_curb_idxes)
        # the left curb slice is shifted further "into" the course along the normal of the curb
        # to provide a margin of safety
        left_curb_slice = left_curb_slice - left_curb_unit_normal * self.CURB_MARGIN

        # generate passing targets by taking the midpoint between NAND and the left curb trajectories
        # the passing target's offset from the nominal is capped at the distance between nominal
        # trajectory and the curb
        passing_targets = (left_curb_slice + other_future_poses) / 2

        # bound the passing targets by the left curb:
        # for curb point C and passing target P, vector P->C dotted with the unit normal of the curb at C
        # should be positive. (the unit normal is left-hand). This ensures the passing points are to the right
        # of the curb
        passing_targets_to_curb = left_curb_slice - passing_targets
        # elementwise multiply, then sum along rows = rowise dot product
        curb_mask = (np.sum(passing_targets_to_curb * left_curb_unit_normal, axis=1) > 0).reshape(-1, 1)

        # to match dimension of passing offsets mask, repeat along row
        # in row (x, y) of passing_targets, if (x, y) is to the left of the curb, the corresponding row
        # in curb_mask = (false, false). Else, the row is (true, true)
        curb_mask = np.repeat(curb_mask, 2, axis=1)

        # use this mask to select the curb, if the original passing target is to the left of the curb.
        passing_targets = np.where(curb_mask, passing_targets, left_curb_slice)

        # bound the passing targets by the nominal trajectory. This means NAND shouldn't "pull"
        # short circuit to the right of the nominal trajectory
        passing_targets_to_nominal = nominal_traj_slice - passing_targets
        nominal_mask = (np.sum(passing_targets_to_nominal * nominal_traj_unit_normal, axis=1) < 0).reshape(-1, 1)
        nominal_mask = np.repeat(nominal_mask, 2, axis=1)
        passing_targets = np.where(nominal_mask, passing_targets, nominal_traj_slice)


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
        # reference_navsat = NavSatFix()
        # ref_gps = World.world_to_gps(*self.nominal_traj.get_position_by_index(int(new_segment_start_idx)))
        # reference_navsat.latitude = ref_gps[0]
        # reference_navsat.longitude = ref_gps[1]
        # self.debug_splice_pt_publisher.publish(reference_navsat)

        # ref_gps = World.world_to_gps(*self.nominal_traj.get_position_by_index(int(new_segment_end_idx)))
        # reference_navsat.latitude = ref_gps[0]
        # reference_navsat.longitude = ref_gps[1]
        # self.debug_splice_pt_publisher.publish(reference_navsat)




        # generate new path
        return Trajectory(json_filepath=None, positions=new_path)