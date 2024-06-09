import numpy as np

from geometry_msgs.msg import Pose as ROSPose
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


class BuggyState:
    """
    A data structure for storing the state of the buggy at a specific timestep (specifically, the UTM position of the buggy, its heading, and its velocity along the easting, northing, and radial directions.

    Also includes helper functions to convert between pose notations.

    """

    __easting = None
    __northing = None
    __yaw = None

    __easting_v = None
    __northing_v = None
    __yaw_v = None


    def __init__(self, east, north, yaw, east_v = 0, north_v = 0, yaw_v = 0):
        self.easting = east
        self.northing = north
        self.yaw = yaw

        self.easting_v = east_v
        self.northing_v = north_v
        self.yaw_v = yaw_v


    def __repr__(self) -> str:
        return f"Pose(easting={self.easting}, northing={self.northing}, yaw={self.yaw}), easting velocity={self.easting_v}, northing velocity={self.northing_v}, yaw velocity={self.yaw_v})"

    def copy(self):
        return BuggyState(self.easting, self.northing, self.yaw, self.easting_v, self.northing_v, self.yaw_v)

    @property
    def easting(self):
        return self.easting

    @__easting.setter
    def easting(self, east):
        self.__easting = east

    @property
    def north(self):
        return self.northing

    @__northing.setter
    def northing(self, north):
        self.__northing = north

    @property
    def yaw(self):
        if self.yaw > np.pi or self.yaw < -np.pi:
            raise ValueError("Yaw is not in [-pi, pi]")
        return self.yaw

    @__yaw.setter
    def yaw(self, yaw):
        # normalize theta to [-pi, pi]
        self.__yaw = np.arctan2(np.sin(yaw), np.cos(yaw))

    @property
    def easting_v(self):
        return self.easting_v

    @__easting_v.setter
    def easting_v(self, eastv):
        self.__easting_v = eastv

    @property
    def northing_v(self):
        return self.northing_v

    @__northing_v.setter
    def easting(self, northv):
        self.__northing_v = northv

    @property
    def yaw_v(self):
        return self.yaw_v

    @__yaw_v.setter
    def easting(self, yawv):
        self.__yaw_v = yawv


    def to_mat(self):
        """
        Returns the pose as a 3x3 homogeneous transformation matrix
        """
        return np.array(
            [
                [np.cos(self.yaw), -np.sin(self.yaw), self.easting],
                [np.sin(self.yaw), np.cos(self.yaw), self.northing],
                [0, 0, 1],
            ]
        )

    # TODO: currently creates state with zero velocity
    @staticmethod
    def from_mat(mat):
        """
        Creates a pose from a 3x3 homogeneous transformation matrix
        """
        return BuggyState(mat[0, 2], mat[1, 2], np.arctan2(mat[1, 0], mat[0, 0]))

    def invert(self):
        """
        Inverts the pose
        """
        return BuggyState.from_mat(np.linalg.inv(self.to_mat()))

    # TODO: do we want all these conversion methods? where are they even used?
    def convert_pose_from_global_to_local_frame(self, pose):
        """
        Converts the inputted pose from the global frame to the local frame
        (relative to the current pose)
        """
        return pose / self

    def convert_pose_from_local_to_global_frame(self, pose):
        """
        Converts the inputted pose from the local frame to the global frame
        (relative to the current pose)
        """
        return pose * self

    def convert_point_from_global_to_local_frame(self, point):
        """
        Converts the inputted point from the global frame to the local frame
        (relative to the current pose)
        """
        point_hg = np.array([point[0], point[1], 1])
        point_hg_local = np.linalg.inv(self.to_mat()) @ point_hg
        return (
            point_hg_local[0] / point_hg_local[2],
            point_hg_local[1] / point_hg_local[2],
        )

    def convert_point_from_local_to_global_frame(self, point):
        """
        Converts the inputted point from the local frame to the global frame
        (relative to the current pose)
        """
        point_hg = np.array([point[0], point[1], 1])
        point_hg_global = self.to_mat() @ point_hg
        return (
            point_hg_global[0] / point_hg_global[2],
            point_hg_global[1] / point_hg_global[2],
        )

    def convert_point_array_from_global_to_local_frame(self, points):
        """
        Converts the inputted point array from the global frame to the local frame
        (relative to the current pose)

        Args:
            points (np.ndarray) [Size: (N,2)]: array of points to convert
        """
        points_hg = np.array([points[:, 0], points[:, 1], np.ones(len(points))])
        points_hg_local = np.linalg.inv(self.to_mat()) @ points_hg.T
        return (points_hg_local[:2, :] / points_hg_local[2, :]).T

    def convert_point_array_from_local_to_global_frame(self, points):
        """
        Converts the inputted point array from the local frame to the global frame
        (relative to the current pose)

        Args:
            points (np.ndarray) [Size: (N,2)]: array of points to convert
        """
        points_hg = np.array([points[:, 0], points[:, 1], np.ones(len(points))])
        points_hg_global = self.to_mat() @ points_hg.T
        return (points_hg_global[:2, :] / points_hg_global[2, :]).T

    def rotate(self, angle):
        """
        Rotates the pose by the given angle
        """
        self.yaw += angle

    def translate(self, x, y):
        """
        Translates the pose by the given x and y distances
        """
        self.x += x
        self.y += y

    def __add__(self, other):
        return BuggyState(self.easting + other.easting, self.northing + other.northing, self.yaw + other.yaw)

    def __sub__(self, other):
        return BuggyState(self.easting - other.easting, self.northing - other.northing, self.yaw - other.yaw)

    def __neg__(self):
        return BuggyState(-self.easting, -self.northing, -self.yaw)

    def __mul__(self, other):
        p1_mat = self.to_mat()
        p2_mat = other.to_mat()

        return BuggyState.from_mat(p1_mat @ p2_mat)

    def __truediv__(self, other):
        p1_mat = self.to_mat()
        p2_mat = other.to_mat()

        return BuggyState.from_mat(np.linalg.inv(p2_mat) @ p1_mat)
