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
# in meters, the number of meters behind NAND before we start morphing the trajectory
REAR_MARGIN = 10

# in meters, the number of meters in front of NAND, 
# before the morphed trajectory rejoins the nominal trajectory
# WARNING: set this value to be greater than 10m/s * lookahead time (10 m/s is the upper limit
# of NAND speed) Failure to do so can result violent u-turns in the new trajectory.

FRONT_MARGIN = 30

class PathPlanner():
    def __init__(self, nominal_traj) -> None:
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
        self.last_cmd = 0
        self.nominal_traj = nominal_traj

        # TODO: estimate this value based on the curvature of NAND's recent positions
        self.other_steering_angle = 0 

    def compute_traj(
        self,
        other_pose: Pose, #Currently NAND's location -- To be Changed
        other_speed: float):

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
            self.other_steering_angle, 
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