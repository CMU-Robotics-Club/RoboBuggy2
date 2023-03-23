from abc import ABC, abstractmethod

import numpy as np
import osqp
import scipy.sparse

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose

from pose import Pose
from trajectory import Trajectory
from controller import Controller
from world import World

import matplotlib.pyplot as plt


class ModelPredictiveController(Controller):
    """
    Convex Model Predictive Controller (MPC)
    """

    DEBUG = False

    # MPC Params
    WHEELBASE = 1.3
    MPC_TIMESTEP = 0.02
    MPC_HORIZON = 100
    LOOKAHEAD_TIME = 0.1  # seconds

    # MPC Cost Weights
    state_cost = 1 * np.array([1, 1, 1, 1])  # x, y, theta, steer
    control_cost = np.array([1])  # d_steer
    final_state_cost = 10.0 * np.array([1, 1, 1, 1])  # x, y, theta, steer

    # State constraints (relative to the reference)
    state_lb = np.array(
        [-np.inf, -np.inf, -2 * np.pi, -np.pi / 9]
    )  # x, y, theta, steer
    state_ub = np.array([np.inf, np.inf, 2 * np.pi, np.pi / 9])  # x, y, theta, steer

    # Control constraints
    control_lb = np.array([-np.pi / 2])  # d_steer
    control_ub = np.array([np.pi / 2])  # d_steer
    # control_lb = np.array([-1000])  # d_steer
    # control_ub = np.array([1000])  # d_steer

    # Internal variables
    current_traj_index = 0  # Where in the trajectory we are currently
    current_speed = 0  # Speed of the buggy used for the MPC calculation. Assumed to be constant for now

    # Optimizer variables
    N_STATES = 4
    N_CONTROLS = 1
    solver: osqp.OSQP = None
    X = np.kron(
        np.eye(MPC_HORIZON),
        np.hstack((np.zeros((N_STATES, N_CONTROLS)), np.eye(N_STATES))),
    )
    U = np.kron(
        np.eye(MPC_HORIZON),
        np.hstack((np.eye(N_CONTROLS), np.zeros((N_CONTROLS, N_STATES)))),
    )

    def __init__(self) -> None:
        self.debug_reference_pos_publisher = rospy.Publisher(
            "auton/debug/reference_navsat", NavSatFix, queue_size=1
        )
        self.debug_error_publisher = rospy.Publisher(
            "auton/debug/error", ROSPose, queue_size=1
        )

        self.solver = osqp.OSQP()

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

        return np.array(
            [
                self.current_speed * np.cos(theta),
                self.current_speed * np.sin(theta),
                self.current_speed * np.tan(steer) / self.WHEELBASE,
                d_steer,
            ]
        )

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

        return np.array(
            [
                [0, 0, -self.current_speed * np.sin(theta), 0],
                [0, 0, self.current_speed * np.cos(theta), 0],
                [
                    0,
                    0,
                    0,
                    self.current_speed / self.WHEELBASE * (1 / np.cos(steer) ** 2),
                ],
                [0, 0, 0, 0],
            ]
        )

    def control_jacobian(self):
        """Return the jacobian of the dynamics function with respect to the control.

        Formula: d_dynamics/d_control
            = [
                [df_x/dd_steer],
                [df_y/dd_steer],
                [df_theta/dd_steer],
                [df_steer/dd_steer]

        Returns:
            np.ndarray: jacobian of the dynamics function
        """
        return np.array([[0], [0], [0], [1]])

    def rotate_state_cost(self, theta):
        """Return the state cost, but rotate the x and y costs by an inputted angle.
        This will allow the cost to be in the frame of the reference trajectory and not the
        global frame.

        Args:
            theta (Number): angle to rotate the cost by in radians
        """
        # return self.state_cost
        return np.abs(
            [
                np.cos(theta) * self.state_cost[0] - np.sin(theta) * self.state_cost[1],
                np.sin(theta) * self.state_cost[0] + np.cos(theta) * self.state_cost[1],
                self.state_cost[2],
                self.state_cost[3],
            ]
        )

    def rotate_final_state_cost(self, theta):
        """Return the final state cost, but rotate the x and y costs by an inputted angle.
        This will allow the cost to be in the frame of the reference trajectory and not the
        global frame.

        Args:
            theta (Number): angle to rotate the cost by in radians
        """
        return np.abs(
            [
                np.cos(theta) * self.final_state_cost[0]
                - np.sin(theta) * self.final_state_cost[1],
                np.sin(theta) * self.final_state_cost[0]
                + np.cos(theta) * self.final_state_cost[1],
                self.final_state_cost[2],
                self.final_state_cost[3],
            ]
        )

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

        # Get reference trajectory from the current traj index for the next [MPC_HORIZON] seconds
        knot_point_distances = np.arange(
            traj_distance,
            traj_distance + self.current_speed * self.MPC_TIMESTEP * self.MPC_HORIZON,
            self.current_speed * self.MPC_TIMESTEP,
        )
        reference_trajectory = np.vstack(
            [
                (
                    *trajectory.get_position_by_distance(d),
                    trajectory.get_heading_by_distance(d),
                    trajectory.get_steering_angle_by_distance(d, self.WHEELBASE),
                )
                for d in knot_point_distances
            ]
        )

        # Build QP matrices
        # We define the problem as follows:
        # z = [u0 x1 u1 x2 ... uN-1 xN]
        # H = [R  0  0  0  ... 0  0
        #      0  Q1 0  0  ... 0  0
        #      0  0  R  0  ... 0  0
        #      0  0  0  Q2 ... 0  0
        #      0  0  0  0  ... R  0
        #      0  0  0  0  ... 0  QN]
        # C = [B0 -I 0  0  0  ... 0    0    0
        #      0  A1 B1 -I 0  ... 0    0    0
        #      0  0  0  A2 B2 ... 0    0    0
        #      0  0  0  0  0  ... 0    0    0
        #      0  0  0  0  0  ... AN-1 BN-1 -I]
        # D = [C; X; U]
        # X selects all the states from z
        # U selects all the controls from z

        # Minimize 0.5 z^T H z + b^T z such that lb <= D z <= ub
        H = scipy.linalg.block_diag(
            np.diag(self.control_cost),
            *[
                m
                for k in range(0, self.MPC_HORIZON - 1)
                for m in [
                    np.diag(self.rotate_state_cost(reference_trajectory[k, 2])),
                    np.diag(self.control_cost),
                ]
            ],
            np.diag(
                self.rotate_final_state_cost(
                    reference_trajectory[self.MPC_HORIZON - 1, 2]
                )
            )
        )
        C = np.block(
            [
                [
                    self.control_jacobian(),
                    -np.eye(self.N_STATES),
                    np.zeros(
                        (
                            self.N_STATES,
                            (self.MPC_HORIZON - 1) * (self.N_STATES + self.N_CONTROLS),
                        )
                    ),
                ],
                *[
                    [
                        np.zeros((self.N_STATES, self.N_CONTROLS)),
                        np.zeros(
                            (
                                self.N_STATES,
                                (k - 1) * (self.N_STATES + self.N_CONTROLS),
                            )
                        ),
                        self.state_jacobian(reference_trajectory[k, :]),
                        self.control_jacobian(),
                        -np.eye(self.N_STATES),
                        np.zeros(
                            (
                                self.N_STATES,
                                (self.MPC_HORIZON - k - 1)
                                * (self.N_STATES + self.N_CONTROLS),
                            )
                        ),
                    ]
                    for k in range(1, self.MPC_HORIZON)
                ],
            ]
        )
        D = np.vstack([C, self.X, self.U])

        lb = np.hstack(
            (
                -self.state_jacobian(reference_trajectory[0, :])
                @ self.state(current_pose.x, current_pose.y, current_pose.theta, 0),
                np.zeros(self.N_STATES * (self.MPC_HORIZON - 1)),
                np.tile(self.state_lb, self.MPC_HORIZON) + reference_trajectory.ravel(),
                np.tile(self.control_lb, self.MPC_HORIZON),
            )
        )
        ub = np.hstack(
            (
                -self.state_jacobian(reference_trajectory[0, :])
                @ self.state(current_pose.x, current_pose.y, current_pose.theta, 0),
                np.zeros(self.N_STATES * (self.MPC_HORIZON - 1)),
                np.tile(self.state_ub, self.MPC_HORIZON) + reference_trajectory.ravel(),
                np.tile(self.control_ub, self.MPC_HORIZON),
            )
        )

        b = np.vstack(
            [
                *[
                    np.hstack(
                        (
                            0,
                            np.diag(-self.rotate_state_cost(reference_trajectory[k, 2]))
                            @ reference_trajectory[k, :],
                        )
                    )
                    for k in range(0, self.MPC_HORIZON - 1)
                ],
                np.hstack(
                    (
                        0,
                        np.diag(
                            -self.rotate_final_state_cost(reference_trajectory[-1, 2])
                        )
                        @ reference_trajectory[self.MPC_HORIZON - 1, :],
                    )
                ),
            ]
        ).ravel()

        # Print shapes of matrices
        if self.DEBUG:
            print("H shape: ", H.shape)
            print("C shape: ", C.shape)
            print("D shape: ", D.shape)
            print("lb shape: ", lb.shape)
            print("ub shape: ", ub.shape)
            print("b shape: ", b.shape)
            print("reference_trajectory shape: ", reference_trajectory.shape)

        # Solve QP
        self.solver.setup(
            P=scipy.sparse.csc_array(H),
            q=b,
            A=scipy.sparse.csc_array(D),
            l=lb,
            u=ub,
            verbose=self.DEBUG,
            eps_abs=1e-8,
            eps_rel=1e-8,
            polish=1,
            time_limit=0.01,
        )
        results = self.solver.solve()
        print(reference_trajectory[0, :])
        steer_angle = results.x[: self.N_CONTROLS]

        if self.DEBUG:
            print("Control: ", steer_angle)

            # Plot the results
            fig, [[ax1, ax2], [ax3, ax4]] = plt.subplots(2, 2)

            # Extract the states
            states = np.zeros((self.MPC_HORIZON, self.N_STATES))
            states[0, :] = self.state(0, 0, 0, 0)
            for k in range(1, self.MPC_HORIZON):
                states[k, :] = results.x[
                    self.N_CONTROLS
                    + k * (self.N_STATES + self.N_CONTROLS) : self.N_CONTROLS
                    + k * (self.N_STATES + self.N_CONTROLS)
                    + self.N_STATES
                ]
            states += reference_trajectory
            ax1.plot(states[:, 0], states[:, 1], "rx-", label="MPC")
            ax1.plot(
                reference_trajectory[:, 0],
                reference_trajectory[:, 1],
                "bx-",
                label="Reference",
            )
            ax1.plot(states[0, 0], states[0, 1], "go", label="Start")
            ax1.plot(states[-1, 0], states[-1, 1], "mo", label="End")

            ax2.plot(states[:, 2], "rx-", label="MPC")
            ax2.plot(reference_trajectory[:, 2], "bx-", label="Reference")

            ax3.plot(states[:, 3], "gx-", label="steer")
            plt.show()

        # Publish error for debugging
        reference_position = trajectory.get_position_by_index(self.current_traj_index)
        reference_error = current_pose.convert_point_from_global_to_local_frame(
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
    ModelPredictiveController.DEBUG = True
    mpc_controller = ModelPredictiveController()

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
