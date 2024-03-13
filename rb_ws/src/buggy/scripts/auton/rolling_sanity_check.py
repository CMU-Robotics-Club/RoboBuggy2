import numpy as np
from nav_msgs.msg import Odometry
from rb_ws.src.buggy.scripts.auton.pose import Pose
from rb_ws.src.buggy.scripts.auton.world import World
from std_msgs.msg import Float32, Float64, Bool, Int8

import rospy

from brake_controller import BrakeController
from auton.controller import Controller
from trajectory import Trajectory
from microstrain_inertial_msgs.msg import FilterStatus, MipFilterStatusGq7StatusFlags
from microstrain_inertial_msgs.msg import SensorOverrangeStatus


class SanityCheck:
    #TODO: copied from autonsystem, delete uneeded args
    global_trajectory: Trajectory = None
    local_controller: Controller = None

    def __init__(self,
            global_trajectory,
            local_controller,
            self_name,
            other_name,
            ) -> None:

        self.global_trajectory = global_trajectory

        #TODO: do we care if there's another buggy actually
        self.has_other_buggy = not other_name is None

        self.cur_traj = global_trajectory
        self.local_controller = local_controller

        self.rtk_status = 0
        self.covariance = 0

        self.location = None
        self.filter_location = None

        self.imu_overrange_status = None
        self.status_flags : MipFilterStatusGq7StatusFlags = None


        #TODO: do we need lock
        self.self_odom_msg = None
        self.other_odom_msg = None

        rospy.Subscriber(self_name + "/nav/odom", Odometry, self.update_self_odom)

        if self.has_other_buggy:
            rospy.Subscriber(other_name + "/nav/odom", Odometry, self.update_other_odom)

        # TODO: do we need these two
        rospy.Subscriber(self_name + "/gnss1/fix_info_republished_int", Int8, self.update_rtk1_status)

        rospy.Subscriber(self_name + "/gnss2/fix_info_republished_int", Int8, self.update_rtk2_status)

        rospy.Subscriber(self_name + "/imu/overrange_status", SensorOverrangeStatus, self.update_overrange_status)

        rospy.Subscriber(self_name + "/nav/status.status_flags", FilterStatus, self.update_status_flags)

    def update_self_odom(self, msg):
        self.self_odom_msg = msg

    def update_other_odom(self, msg):
        self.other_odom_msg = msg

    def update_overrange_status(self, msg : SensorOverrangeStatus):
        self.imu_overrange_status = msg.status

    def update_status_flags(self, msg : FilterStatus):
        self.status_flags = msg.gq7_status_flags #TODO: do we have gx5 or gq7??
        # also what if we just published these? seems easy to read do we not publish this
        #they are NOT bools it IS a number




    def calc_covariance(self):
        self.covariance = self.self_odom_msg.pose.covariance[0] ** 2 + self.self_odom_msg.pose.covariance[7] ** 2

    def calc_locations(self):
        current_rospose = self.self_odom_msg.pose.pose
        current_speed = np.sqrt(
            self.self_odom_msg.twist.twist.linear.x**2 + self.self_odom_msg.twist.twist.linear.y**2
        )
        pose_gps = Pose.rospose_to_pose(current_rospose)
        self.location = World.gps_to_world_pose(pose_gps)

        self.filter_location = None #TODO: where does ins publish its location

    def is_overrange (self):
        #TODO: replace with wiht bools we care about
        # http://docs.ros.org/en/api/microstrain_inertial_msgs/html/msg/MipSensorOverrangeStatus.html)
        # if ANY are true return true
        s = self.imu_overrange_status
        return s.accel_x # look at all the foxglove outputs

    def filter_status_warning (self):
        #TODO: replace with wiht bools we care about
        # http://docs.ros.org/en/api/microstrain_inertial_msgs/html/msg/MipFilterStatusGq7StatusFlags.html
        return self.status_flags.roll_pitch_warning


    # TODO: add docs to this (this is essentially main)
    def sanity_check(self):
        if (abs(self.filter_location - self.location) > 0):
            print("filter and gps seperate")

        if (self.covariance > 1**2):
            print("covariance bad")

        if (self.is_overrange):
            print("overrange warning raised")

        if (self.filter_status_warning):
            print("filter unhappy: roll pitch warning raised")
            # TODO: publish the relevant everything


        # TODO: add node to launchfiles

