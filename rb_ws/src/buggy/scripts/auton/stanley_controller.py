import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose
from nav_msgs.msg import Odometry


from pose import Pose
from trajectory import Trajectory
from controller import Controller
from world import World


class StanleyController(Controller):
    """
    Stanley Controller (front axle used as reference point)
    Referenced from this paper: https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf
    """

    LOOK_AHEAD_DIST_CONST = 0.05 # s
    MIN_LOOK_AHEAD_DIST = 0.1
    MAX_LOOK_AHEAD_DIST = 2.0

    CROSS_TRACK_GAIN = 1.3
    K_SOFT = 1.0 # m/s
    K_D_YAW = 0.012 # rad / (rad/s)

    def __init__(self, buggy_name, start_index=0) -> None:
        super(StanleyController, self).__init__(start_index, buggy_name)
        self.debug_reference_pos_publisher = rospy.Publisher(
            buggy_name + "/auton/debug/reference_navsat", NavSatFix, queue_size=1
        )
        self.debug_error_publisher = rospy.Publisher(
            buggy_name + "/auton/debug/error", ROSPose, queue_size=1
        )

    def compute_control(
        self, state_msg: Odometry, trajectory: Trajectory
    ):
        """Computes the steering angle determined by Stanley controller.
        Does this by looking at the crosstrack error + heading error

        Args:
            state_msg: ros Odometry message
            trajectory (Trajectory): reference trajectory
            yaw_rate (float): current yaw rate of the buggy (rad/s)

        Returns:
            float (desired steering angle)
        """
        if self.current_traj_index >= trajectory.get_num_points() - 1:
            raise Exception("[Stanley]: Ran out of path to follow!")

        current_rospose = state_msg.pose.pose
        current_pose = World.gps_to_world_pose(Pose.rospose_to_pose(current_rospose))
        current_speed = np.sqrt(
            state_msg.twist.twist.linear.x**2 + state_msg.twist.twist.linear.y**2
        )
        yaw_rate = state_msg.twist.twist.angular.z

        heading = current_pose.theta  # in radians
        x = current_pose.x
        y = current_pose.y

        # Assume current pose is rear of buggy, project it to center of front axle
        front_x = x + StanleyController.WHEELBASE * np.cos(heading)
        front_y = y + StanleyController.WHEELBASE * np.sin(heading)

        # setting range of indices to search so we don't have to search the entire path
        traj_index = trajectory.get_closest_index_on_path(
            front_x,
            front_y,
            start_index=self.current_traj_index - 20,
            end_index=self.current_traj_index + 50,
        )
        self.current_traj_index = max(traj_index, self.current_traj_index)

        # Calculate heading error

        lookahead_dist = np.clip(
            self.LOOK_AHEAD_DIST_CONST * current_speed,
            self.MIN_LOOK_AHEAD_DIST,
            self.MAX_LOOK_AHEAD_DIST)

        traj_dist = trajectory.get_distance_from_index(self.current_traj_index) + lookahead_dist

        ref_heading = trajectory.get_heading_by_index(
            trajectory.get_index_from_distance(traj_dist))

        error_heading = ref_heading - current_pose.theta
        error_heading = np.arctan2(np.sin(error_heading), np.cos(error_heading))

        # Calculate cross track error by finding the distance from the buggy to the tangent line of
        # the reference trajectory
        closest_position = trajectory.get_position_by_index(self.current_traj_index)
        next_position = trajectory.get_position_by_index(
            self.current_traj_index + 0.0001
        )
        x1 = closest_position[0]
        y1 = closest_position[1]
        x2 = next_position[0]
        y2 = next_position[1]
        error_dist = -((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)) / np.sqrt(
            (y2 - y1) ** 2 + (x2 - x1) ** 2
        )

        speed = current_speed
        cross_track_component = -np.arctan2(
            StanleyController.CROSS_TRACK_GAIN * error_dist, speed + StanleyController.K_SOFT
        )

        # Calculate yaw rate error
        r_meas = yaw_rate
        r_traj = speed * (trajectory.get_heading_by_index(trajectory.get_index_from_distance(traj_dist) + 0.05)
        - trajectory.get_heading_by_index(trajectory.get_index_from_distance(traj_dist))) / 0.05


        steering_cmd = error_heading + cross_track_component + StanleyController.K_D_YAW * (r_traj - r_meas)
        steering_cmd = np.clip(steering_cmd, -np.pi / 9, np.pi / 9)

        reference_position = trajectory.get_position_by_index(self.current_traj_index)
        reference_error = current_pose.convert_point_from_global_to_local_frame(
            reference_position
        )
        reference_error -= np.array([StanleyController.WHEELBASE, 0])

        # Publish error for debugging
        error_pose = ROSPose()
        error_pose.position.x = reference_error[0]
        error_pose.position.y = reference_error[1]
        self.debug_error_publisher.publish(error_pose)

        # Publish reference position for debugging
        reference_navsat = NavSatFix()
        ref_gps = World.world_to_gps(*closest_position)
        reference_navsat.latitude = ref_gps[0]
        reference_navsat.longitude = ref_gps[1]
        self.debug_reference_pos_publisher.publish(reference_navsat)

        return steering_cmd
