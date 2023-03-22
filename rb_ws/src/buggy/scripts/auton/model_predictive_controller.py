from abc import ABC, abstractmethod

import numpy as np

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose

from pose import Pose
from trajectory import Trajectory
from controller import Controller
from world import World


class ModelPredictiveController(Controller):
    """
    Convex Model Predictive Controller (MPC)
    """

    DEBUG = False

    # MPC Params
    WHEELBASE = 1.3
    MPC_TIMESTEP = 0.1
    MPC_HORIZON = 10
    LOOKAHEAD_TIME = 0.1  # seconds

    # MPC Cost Weights
    state_cost = np.diag([0, 1, 0.1, 0.01]) # x, y, theta, steer
    control_cost = np.diag([1]) # d_steer
    final_state_cost = 10.0 * np.diag([0, 1, 0.1, 0.01]) # x, y, theta, steer

    # Internal variables
    current_traj_index = 0  # Where in the trajectory we are currently
    current_speed = 0  # Speed of the buggy used for the MPC calculation. Assumed to be constant for now

    def __init__(self) -> None:
        self.debug_reference_pos_publisher = rospy.Publisher(
            "auton/debug/reference_navsat", NavSatFix, queue_size=1
        )
        self.debug_error_publisher = rospy.Publisher(
            "auton/debug/error", ROSPose, queue_size=1
        )

    def state(self, x, y, theta, steer):
        """Return the state vector. This describes where the buggy currently is (plus some
        other information)

        Args:
            x (Number): x coordinate of the buggy in meters
            y (Number): y coordinate of the buggy in meters
            theta (Number): heading in radians of the buggy (zero is along the +x axis)
            steer (Number): steering angle in radians of the front wheel (zero is straight, + is left)

        Returns:
            np.ndarray: state vector used by the rest of the codebase
        """
        return np.array([x, y, theta, steer])
    
    def control(self, d_steer):
        """Return the control vector. This describes the control input to the buggy

        Args:
            d_steer (Number): change in steering angle in radians of the front wheel (zero is no change, + is accelerating left)

        Returns:
            np.ndarray: control vector used by the rest of the codebase
        """
        return np.array([d_steer])
    
    def dynamics(self, state, control):
        """Return the dynamics of the system. This is used to model the system such that
        the optimizer (QP Solver) can return an optimal trajectory.

        Formula: dynamics = d_state/dt = f(state, control)

        Args:
            state (np.ndarray): state vector
            control (np.ndarray): control vector

        Notes:
            self.current_speed must be set before calling this function
        
        Returns:
            np.ndarray: dynamics vector
        """
        _, _, theta, steer = state
        d_steer = control

        return np.array([
            self.current_speed * np.cos(theta),
            self.current_speed * np.sin(theta),
            self.current_speed * np.tan(steer) / self.WHEELBASE,
            d_steer
        ])
    
    def state_jacobian(self, state):
        """Return the jacobian of the dynamics function with respect to the state. 

        Formula: df/d_state
            = [
                [df_x/dx, df_x/dy, df_x/dtheta, df_x/dsteer],
                [df_y/dx, df_y/dy, df_y/dtheta, df_y/dsteer],
                [df_theta/dx, df_theta/dy, df_theta/dtheta, df_theta/dsteer],
                [df_steer/dx, df_steer/dy, df_steer/dtheta, df_steer/dsteer]
            ]

        Args:
            state (np.ndarray): state vector

        Returns:
            np.ndarray: jacobian of the dynamics function
        """
        _, _, theta, steer = state

        return np.array([
            [0, 0, -self.current_speed * np.sin(theta), 0],
            [0, 0, self.current_speed * np.cos(theta), 0],
            [0, 0, 0, self.current_speed / self.WHEELBASE * (1 / np.cos(steer)**2)],
            [0, 0, 0, 0]
        ])
    
    def control_jacobian(self):
        """Return the jacobian of the dynamics function with respect to the control. 

        Formula: d_dynamics/d_control
            = [
                [df_x/dd_steer],
                [df_y/dd_steer],
                [df_theta/dd_steer],
                [df_steer/dd_steer]

        Args:
            state (np.ndarray): state vector

        Returns:
            np.ndarray: jacobian of the dynamics function
        """
        return np.array([
            [0],
            [0],
            [0],
            [1]
        ])

    def compute_control(
        self,
        current_pose: Pose,
        trajectory: Trajectory,
        current_velocity: Pose,
    ):
        """
        Computes the desired control output given the current state and reference trajectory

        Args:
            current_pose (Pose): current pose (x, y, theta) (UTM coordinates)
            trajectory (Trajectory): reference trajectory
            current_velocity (Pose): current velocity of the buggy (dx, dy, dtheta)

        Returns:
            float (desired steering angle)
        """
        # Get reference pose
        if self.current_traj_index >= trajectory.get_num_points() - 1:
            return 0

        self.current_speed = np.sqrt(current_velocity.x**2 + current_velocity.y**2)

        traj_index = trajectory.get_closest_index_on_path(
            current_pose.x,
            current_pose.y,
            start_index=self.current_traj_index,
            end_index=self.current_traj_index + 10,
        )
        self.current_traj_index = max(traj_index, self.current_traj_index)
        traj_distance = trajectory.get_distance_from_index(self.current_traj_index)
        traj_distance += self.LOOKAHEAD_TIME * self.current_speed

        steer_angle = 0

        if self.DEBUG:
            print("Control: ", steer_angle)

        # Publish error for debugging
        reference_position = trajectory.get_position_by_index(self.current_traj_index)
        reference_error = self.current_pose.convert_point_from_global_to_local_frame(
            reference_position
        )
        error_pose = ROSPose()
        error_pose.position.x = reference_error[0]
        error_pose.position.y = reference_error[1]
        self.debug_error_publisher.publish(error_pose)

        # Publish reference position for debugging
        reference_navsat = NavSatFix()
        ref_gps = World.world_to_gps(*reference_position)
        # ref_gps = World.world_to_gps(current_pose.x, current_pose.y)
        reference_navsat.latitude = ref_gps[0]
        reference_navsat.longitude = ref_gps[1]
        self.debug_reference_pos_publisher.publish(reference_navsat)

        return steer_angle


if __name__ == "__main__":
    # ModelPredictiveController.DEBUG = True
    mpc_controller = ModelPredictiveController(test_only=True)

    trajectory = Trajectory("/rb_ws/src/buggy/paths/quartermiletrack.json")
    end_pose = Pose(
        *trajectory.get_position_by_distance(25 * 0.02 * 5),
        trajectory.get_heading_by_distance(25 * 0.02 * 5)
    )
    print(end_pose)

    # Offset initial pose slightly
    initial_pose = Pose(
        *trajectory.get_position_by_distance(0), trajectory.get_heading_by_distance(0)
    )
    initial_pose.x += 1
    initial_pose.y += 0.25
    initial_pose.theta += 0.1

    mpc_controller.compute_control(initial_pose, trajectory, Pose(5, 0, 0))
