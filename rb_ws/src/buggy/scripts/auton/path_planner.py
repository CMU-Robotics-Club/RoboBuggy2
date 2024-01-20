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
NOMINAL_STEERING_ERR_GAIN = 0.001
MAX_STEERING_ANGLE = 20
OTHER_BUGGY_COST = 1
other_steering_angle = 0 # TODO: update this variable with NAND steering angle topic

class PathPlanner():
    def __init__(self) -> None:
        self.occupancy_grid = OccupancyGrid()
        self.path_projector = Projector(1.3)

        # in degrees
        self.candidate_steering_angles = np.linspace(-5, 5, num=20)

        # range of possible change in steering angle of other buggy
        self.other_steering_angles = np.linspace(-1, 1, num=2)

        self.debug_local_traj_publisher = rospy.Publisher(
            "/auton/debug/reference_traj", NavSatFix, queue_size=1000
        )

        self.debug_grid_cost_publisher = rospy.Publisher(
            "/auton/debug/grid_cost", Float64, queue_size=0
        )
        self.last_cmd = 0

    def compute_steering_angle_max_estimate(self, velocity):
        return np.min(MAX_STEERING_ANGLE / velocity)

    def compute_traj(
        self, current_pose: Pose, 
        other_pose: Pose,
        current_speed: float,
        other_speed: float,
        nominal_steering_angle: float,
        other_steering_angle: float):

        # add other buggy's projected trajectory to the grid as no-go regions

        # TODO: add nand steering angle topic in Simulator3 and on FW
        # project nand's position to end of horizon
    
        other_steering_angles = self.other_steering_angles + other_steering_angle
        other_pose_utm = World.world_to_utm_pose(other_pose)

        for angle in other_steering_angles:
            other_future_poses = self.path_projector.project(
                other_pose_utm, 
                angle,
                other_speed, 
                LOOKAHEAD_TIME,
                RESOLUTION).tolist()
    
            self.occupancy_grid.set_cost(other_future_poses, OTHER_BUGGY_COST)

        
        pose_dummy_left_utm = copy.deepcopy(other_pose_utm)
        pose_dummy_left_utm.rotate(np.pi/2)

        pose_dummy_right_utm = copy.deepcopy(other_pose_utm)
        pose_dummy_right_utm.rotate(-np.pi/2)
        
        side_padding_r = self.path_projector.project(
            pose_dummy_right_utm,
            0,
            1,
            20,
            3
        ).tolist()
        
        side_padding_l = self.path_projector.project(
            pose_dummy_left_utm,
            0,
            1,
            3,
            3
        ).tolist()

        self.occupancy_grid.set_cost(side_padding_l, np.linspace(OTHER_BUGGY_COST, 0, len(side_padding_l)))
        self.occupancy_grid.set_cost(side_padding_r, OTHER_BUGGY_COST)

        min_cost = None
        best_traj = None
        best_cmd = nominal_steering_angle

        
        # for each candidate steering angle, call projector
        # TODO: explore generating path trees 
        # self.candidate_steering_angles.append(nominal_steering_angle)
        candidate_steering_angles = (self.candidate_steering_angles).tolist()

        for steering_angle in candidate_steering_angles:
            # canidate_traj is in world coords, and is a 2d list
            candidate_traj = self.path_projector.project(current_pose, steering_angle, current_speed, LOOKAHEAD_TIME, RESOLUTION)
            candidate_traj_utm = World.world_to_utm_numpy(candidate_traj)

            # calculate grid cost for this trajectory
            grid_cost = self.occupancy_grid.get_utm_cost(candidate_traj_utm)
            if (grid_cost > 0):
                print(grid_cost)

            # tiebreak in favor of s†eering angle that is closer to the nominal action 
            # (the steering angle that we should use if there was no other buggy)
            nominal_steering_err = NOMINAL_STEERING_ERR_GAIN * abs(nominal_steering_angle - steering_angle)

            cost = nominal_steering_err + grid_cost 
            # compare cost
            if min_cost is None or cost < min_cost:
                best_traj = candidate_traj
                min_cost = cost
                best_cmd = steering_angle
        # self.candidate_steering_angles.pop()

        for i in range(len(best_traj)):
            reference_navsat = NavSatFix()
            ref_gps = World.world_to_gps(*best_traj[i])
            reference_navsat.latitude = ref_gps[0]
            reference_navsat.longitude = ref_gps[1]
            self.debug_local_traj_publisher.publish(reference_navsat)

        for i in range(len(side_padding_l)):
            reference_navsat = NavSatFix()
            ref_gps = World.utm_to_gps(*side_padding_l[i])
            reference_navsat.latitude = ref_gps[0]
            reference_navsat.longitude = ref_gps[1]
            self.debug_local_traj_publisher.publish(reference_navsat)

        for i in range(len(side_padding_r)):
            reference_navsat = NavSatFix()
            ref_gps = World.utm_to_gps(*side_padding_r[i])
            reference_navsat.latitude = ref_gps[0]
            reference_navsat.longitude = ref_gps[1]
            self.debug_local_traj_publisher.publish(reference_navsat)

        # for i in range(len(other_future_poses)):
        #     reference_navsat = NavSatFix()
        #     ref_gps = World.utm_to_gps(*other_future_poses[i])
        #     reference_navsat.latitude = ref_gps[0]
        #     reference_navsat.longitude = ref_gps[1]
        #     self.debug_local_traj_publisher.publish(reference_navsat)


        

        self.occupancy_grid.reset_grid()

        self.last_cmd = best_cmd

        return best_cmd
        # return traj_object

