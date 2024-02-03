import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose as ROSPose
from pose import Pose

from occupancy_grid.grid_manager import OccupancyGrid
from path_projection import Projector
from trajectory import Trajectory
from world import World
import copy


LOOKAHEAD_TIME = 2.0 #s
RESOLUTION = 30 #samples/s
PASSING_OFFSET = 2 #m
NOMINAL_STEERING_ERR_GAIN = 0.001
MAX_STEERING_ANGLE = 20
OTHER_BUGGY_COST = 1
other_steering_angle = 0 # TODO: update this variable with NAND steering angle topic

class PathPlanner():
    def __init__(self, nominal_traj) -> None:
        self.occupancy_grid = OccupancyGrid()
        self.path_projector = Projector(1.3)

        # in degrees
        self.candidate_steering_angles = np.linspace(-5, 5, num=20)

        # range of possible change in steering angle of other buggy
        self.other_steering_angles = np.linspace(-1, 1, num=2)

        self.debug_passing_traj_publisher = rospy.Publisher(
            "/auton/debug/passing_traj", NavSatFix, queue_size=1000
        )

        self.debug_splice_pt_publisher = rospy.Publisher(
            "/auton/debug/splice_pts", NavSatFix, queue_size=1000
        )


        self.debug_grid_cost_publisher = rospy.Publisher(
            "/auton/debug/grid_cost", Float64, queue_size=0
        )
        self.last_cmd = 0
        self.nominal_traj = nominal_traj

    def compute_steering_angle_max_estimate(self, velocity):
        return np.min(MAX_STEERING_ANGLE / velocity)

    def compute_traj(
        self, current_pose: Pose, 
        other_pose: Pose, #Currently NAND's location -- To be Changed
        current_speed: float,
        other_speed: float,
        nominal_steering_angle: float,
        other_steering_angle: float):

        # draw trajectory, such that the section of the
        # trajectory near NAND is replace by a new segment:

        # 1. the global path, at 10m behind NAND
        # 2. NAND's projected locations, where each location is 
        # shifted to the left along the normal of the trajectory vector

        other_idx = self.nominal_traj.get_closest_index_on_path(
            other_pose.x, 
            other_pose.y)
        #other is just NAND, for general purposes consider it other

        new_segment_start_idx = self.nominal_traj.get_index_from_distance(
                self.nominal_traj.get_distance_from_index(other_idx) - 10
            ) #Where new path index starts, CONSTANT delta from NAND
        
        new_segment_end_idx = self.nominal_traj.get_index_from_distance(
                self.nominal_traj.get_distance_from_index(other_idx) + 30
            )
        
        # project other buggy path
        # TODO: put other buggy command
        other_future_poses = self.path_projector.project(
            other_pose,
            0, 
            other_speed, 
            LOOKAHEAD_TIME, 
            RESOLUTION)

        other_future_poses_idxs = np.empty((len(other_future_poses), ))

        # TODO: optimize this lookup -- how tho
        for i in range(len(other_future_poses)):
            other_future_poses_idxs[i] = self.nominal_traj.get_closest_index_on_path(
                    other_future_poses[i][0],
                    other_future_poses[i][1],
                    start_index=other_idx)
            
        future_pose_unit_normal = self.nominal_traj.get_unit_normal_by_index(
            other_future_poses_idxs
        )

        # the first passing target is 10 meters backward from NAND's position
        passing_targets = np.array(
            [self.nominal_traj.get_position_by_index(new_segment_start_idx)])
        
        passing_targets = np.vstack((passing_targets, 
            other_future_poses + PASSING_OFFSET * future_pose_unit_normal))
        
        
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
    
        # for i in range(len(best_traj)):
        #     reference_navsat = NavSatFix()
        #     ref_gps = World.world_to_gps(*best_traj[i])
        #     reference_navsat.latitude = ref_gps[0]
        #     reference_navsat.longitude = ref_gps[1]
        #     self.debug_local_traj_publisher.publish(reference_navsat)

        # for i in range(len(side_padding_l)):
        #     reference_navsat = NavSatFix()
        #     ref_gps = World.utm_to_gps(*side_padding_l[i])
        #     reference_navsat.latitude = ref_gps[0]
        #     reference_navsat.longitude = ref_gps[1]
        #     self.debug_local_traj_publisher.publish(reference_navsat)

        # for i in range(len(side_padding_r)):
        #     reference_navsat = NavSatFix()
        #     ref_gps = World.utm_to_gps(*side_padding_r[i])
        #     reference_navsat.latitude = ref_gps[0]
        #     reference_navsat.longitude = ref_gps[1]
        #     self.debug_local_traj_publisher.publish(reference_navsat)

        # for i in range(len(other_future_poses)):
        #     reference_navsat = NavSatFix()
        #     ref_gps = World.utm_to_gps(*other_future_poses[i])
        #     reference_navsat.latitude = ref_gps[0]
        #     reference_navsat.longitude = ref_gps[1]
        #     self.debug_local_traj_publisher.publish(reference_navsat)

        # self.occupancy_grid.reset_grid()

        # self.last_cmd = best_cmd

        # return best_cmd

