import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose
from pose import Pose

import occupancy_grid
from occupancy_grid.grid_manager import OccupancyGrid
from path_projection import Projector

class PathPlanner():
    def __init__(self) -> None:
        self.occupancy_grid = OccupancyGrid()
        self.path_projector = Projector()

    def compute_traj(
        self, current_pose: Pose, trajectory: Trajectory, current_speed: float):
