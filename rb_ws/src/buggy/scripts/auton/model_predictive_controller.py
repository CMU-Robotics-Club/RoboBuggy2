from abc import ABC, abstractmethod

import time

import numpy as np
import osqp
import scipy
from scipy import sparse

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
    PLOT = False
    TIME = False

    # MPC Params
    WHEELBASE = 1.3
    MPC_TIMESTEP = 0.02
    MPC_HORIZON = 50
    LOOKAHEAD_TIME = 0.05  # seconds
    N_STATES = 4
    N_CONTROLS = 1

    # MPC Cost Weights
    state_cost = np.array([0.0001, 20, 1, 1])  # x, y, theta, steer
    control_cost = np.array([1])  # d_steer
    final_state_cost = 10.0 * np.array([0.0001, 20, 1, 1])  # x, y, theta, steer

    # State constraints (relative to the reference)
    state_lb = np.array([-np.inf, -np.inf, -np.inf, -np.pi / 9])  # x, y, theta, steer
    state_ub = np.array([np.inf, np.inf, np.inf, np.pi / 9])  # x, y, theta, steer

    # Control constraints
    # control_lb = np.array([-np.pi / 2])  # d_steer
    # control_ub = np.array([np.pi / 2])  # d_steer
    control_lb = np.array([-np.inf])  # d_steer
    control_ub = np.array([np.inf])  # d_steer

    # Internal variables
    current_traj_index = 0  # Where in the trajectory we are currently
    current_speed = 0  # Speed of the buggy used for the MPC calculation. Assumed to be constant for now

    # Optimizer variables
    solver: osqp.OSQP = None
    solver_settings: dict = {
        "verbose": DEBUG,
        "eps_abs": 1e-4,
        "eps_rel": 1e-4,
        "polish": 1,
        # "time_limit": 0.01,
        "warm_start": False,
    }
    X = sparse.kron(
        np.eye(MPC_HORIZON),
        np.hstack((np.zeros((N_STATES, N_CONTROLS)), np.eye(N_STATES))),
        format="csc",
    )
    U = sparse.kron(
        np.eye(MPC_HORIZON),
        np.hstack((np.eye(N_CONTROLS), np.zeros((N_CONTROLS, N_STATES)))),
        format="csc",
    )
    b = np.zeros(MPC_HORIZON * (N_STATES + N_CONTROLS))

    def __init__(self) -> None:
        self.debug_reference_pos_publisher = rospy.Publisher(
            "auton/debug/reference_navsat", NavSatFix, queue_size=1
        )
        self.debug_error_publisher = rospy.Publisher(
            "auton/debug/error", ROSPose, queue_size=1
        )

        self.solver = osqp.OSQP()

        self.C = sparse.kron(
            np.eye(self.MPC_HORIZON),
            np.hstack((self.control_jacobian(), -np.eye(self.N_STATES))),
            format="csc",
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
        ) * self.MPC_TIMESTEP + np.eye(self.N_STATES)

    def control_jacobian(self):
        """Return the jacobian of the dynamics function with respect to the control.

        Formula: d_dynamics/d_control
            = [
                [df_x/dd_steer],
                [df_y/dd_steer],
                [df_theta/dd_steer],
                [df_steer/dd_steer],
            ]

        Returns:
            np.ndarray: jacobian of the dynamics function
        """
        return np.array([[0], [0], [0], [1]]) * self.MPC_TIMESTEP

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
        # return self.final_state_cost
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

    first_iteration = True

    def compute_control(
        self,
        current_pose: Pose,
        trajectory: Trajectory,
        current_speed: float,
    ):
        """
        Computes the desired control output given the current state and reference trajectory

        Args:
            current_pose (Pose): current pose (x, y, theta) (UTM coordinates)
            trajectory (Trajectory): reference trajectory
            current_speed (float): current speed of the buggy

        Returns:
            float (desired steering angle)
        """
        if self.TIME:
            print("--------------------")
            totaltime = time.time()

        # Get reference pose
        if self.current_traj_index >= trajectory.get_num_points() - 1:
            return 0

        if self.TIME:
            t = time.time()
        self.current_speed = current_speed

        traj_index = trajectory.get_closest_index_on_path(
            current_pose.x,
            current_pose.y,
            start_index=self.current_traj_index,
            end_index=self.current_traj_index + 100,
        )
        self.current_traj_index = max(traj_index, self.current_traj_index)
        traj_distance = trajectory.get_distance_from_index(self.current_traj_index)
        traj_distance += self.LOOKAHEAD_TIME * self.current_speed

        if self.DEBUG:
            print("Current traj index: ", self.current_traj_index)

        # Get reference trajectory from the current traj index for the next [MPC_HORIZON] seconds
        knot_point_distances = np.linspace(
            traj_distance,
            traj_distance + self.current_speed * self.MPC_TIMESTEP * self.MPC_HORIZON,
            self.MPC_HORIZON,
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
        # print(knot_point_distances)
        reference_control = np.gradient(reference_trajectory[:, 3]) / self.MPC_TIMESTEP

        if self.TIME:
            print("Get reference trajectory: ", time.time() - t)

        # print("Current pose: ", current_pose)
        # print("Reference pose: ", reference_trajectory[0, :])

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

        # Minimize 0.5 z^T H z such that lb <= D z <= ub
        if self.TIME:
            t = time.time()

        H = sparse.block_diag(
            (
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
                ),
            ),
            format="csc",
        )

        C1 = sparse.hstack(
            (
                sparse.block_diag(
                    (
                        np.zeros((self.N_STATES, self.N_CONTROLS)),
                        *[
                            np.hstack(
                                (
                                    self.state_jacobian(reference_trajectory[k, :]),
                                    np.zeros((self.N_STATES, self.N_CONTROLS)),
                                )
                            )
                            for k in range(1, self.MPC_HORIZON)
                        ],
                    ),
                    format="csc",
                ),
                np.zeros((self.N_STATES * self.MPC_HORIZON, self.N_STATES)),
            )
        )
        D = sparse.vstack([self.C + C1, self.X, self.U])

        lb = np.hstack(
            (
                -self.state_jacobian(reference_trajectory[0, :])
                @ (
                    self.state(current_pose.x, current_pose.y, current_pose.theta, 0)
                    - reference_trajectory[0, :]
                ),
                np.zeros(self.N_STATES * (self.MPC_HORIZON - 1)),
                # reference_trajectory[2:, :].ravel()
                # - reference_trajectory[1:-1, :].ravel(),
                # np.zeros(self.N_STATES),
                # np.repeat(-np.inf, 4),
                # self.state(current_pose.x, current_pose.y, current_pose.theta, 0),
                np.tile(self.state_lb, self.MPC_HORIZON) + reference_trajectory.ravel(),
                np.tile(self.control_lb, self.MPC_HORIZON) + reference_control.ravel(),
            )
        )
        ub = np.hstack(
            (
                -self.state_jacobian(reference_trajectory[0, :])
                @ (
                    self.state(current_pose.x, current_pose.y, current_pose.theta, 0)
                    - reference_trajectory[0, :]
                ),
                np.zeros(self.N_STATES * (self.MPC_HORIZON - 1)),
                # reference_trajectory[2:, :].ravel()
                # - reference_trajectory[1:-1, :].ravel(),
                # np.zeros(self.N_STATES),
                # np.repeat(np.inf, 4),
                # self.state(current_pose.x, current_pose.y, current_pose.theta, 0),
                np.tile(self.state_ub, self.MPC_HORIZON) + reference_trajectory.ravel(),
                np.tile(self.control_ub, self.MPC_HORIZON) + reference_control.ravel(),
            )
        )

        if self.TIME:
            print(" Create matrices: ", 1000 * (time.time() - t))

        # Print shapes of matrices
        if self.DEBUG:
            print("H shape: ", H.shape)
            print("C shape: ", self.C.shape)
            print("D shape: ", D.shape)
            print("lb shape: ", lb.shape)
            print("ub shape: ", ub.shape)
            print("b shape: ", self.b.shape)
            print("reference_trajectory shape: ", reference_trajectory.shape)
            print("reference_control shape: ", reference_control.shape)

        # Solve QP

        if self.TIME:
            t = time.time()
        if self.first_iteration:
            self.solver.setup(
                **self.solver_settings,
                P=H,
                q=self.b,
                A=D,
                l=lb,
                u=ub,
            )
            self.first_iteration = False
        else:
            self.solver.update(
                Px=sparse.triu(H).data,
                Ax=D.data,
                l=lb,
                u=ub,
            )
        if self.TIME:
            print(" Setup: ", 1000 * (time.time() - t))

        if self.TIME:
            t = time.time()
        results = self.solver.solve()
        steer_angle = results.x[self.N_CONTROLS + self.N_STATES - 1]
        if self.TIME:
            print(" Solve: ", 1000 * (time.time() - t))

        if steer_angle == None:
            print("No solution found")

        if self.DEBUG:
            print("Control: ", steer_angle)

        if self.PLOT:
            # Plot the results
            fig, [[ax1, ax2], [ax3, ax4]] = plt.subplots(2, 2)

            # Extract the states
            states = np.zeros((self.MPC_HORIZON, self.N_STATES))
            for k in range(0, self.MPC_HORIZON):
                states[k, :] = results.x[
                    self.N_CONTROLS
                    + k * (self.N_STATES + self.N_CONTROLS) : self.N_CONTROLS
                    + k * (self.N_STATES + self.N_CONTROLS)
                    + self.N_STATES
                ]
            states += reference_trajectory
            controls = np.zeros((self.MPC_HORIZON, self.N_CONTROLS))
            for k in range(0, self.MPC_HORIZON):
                controls[k, :] = results.x[
                    k
                    * (self.N_STATES + self.N_CONTROLS) : k
                    * (self.N_STATES + self.N_CONTROLS)
                    + self.N_CONTROLS
                ]
            controls += reference_control.reshape((-1, 1))

            # print(states[:10, :])
            # s3 = states[3, :] - reference_trajectory[3, :]
            # s2 = states[2, :] - reference_trajectory[2, :]
            # s1 = states[1, :] - reference_trajectory[1, :]
            # s0 = states[0, :] - reference_trajectory[0, :]
            # print(
            #     s3
            #     - self.state_jacobian(reference_trajectory[2, :]) @ (s2)
            #     - self.control_jacobian() @ (controls[2, :] - reference_control[2])
            # )
            # print(
            #     s2
            #     - self.state_jacobian(reference_trajectory[1, :]) @ (s1)
            #     - self.control_jacobian() @ (controls[1, :] - reference_control[1])
            # )
            # print(
            #     s1
            #     - self.state_jacobian(reference_trajectory[0, :]) @ (s0)
            #     - self.control_jacobian() @ (controls[0, :] - reference_control[0])
            # )
            # print(
            #     s0
            #     - self.state_jacobian(
            #         self.state(current_pose.x, current_pose.y, current_pose.theta, 0)
            #     )
            #     @ (s0)
            # )

            ax1.set_title("Position (red=mpc, blue=ref)")
            ax1.plot(states[:, 0], states[:, 1], "rx-", label="MPC")
            ax1.plot(
                reference_trajectory[:, 0],
                reference_trajectory[:, 1],
                "bx-",
                label="Reference",
            )
            ax1.plot(states[0, 0], states[0, 1], "go", label="Start")
            ax1.plot(states[-1, 0], states[-1, 1], "mo", label="End")

            ax2.set_title("Heading")
            ax2.plot(states[:, 2], "rx-", label="MPC")
            ax2.plot(reference_trajectory[:, 2], "bx-", label="Reference")

            ax3.set_title("Steering angle")
            ax3.plot(states[:, 3], "rx-", label="MPC")
            ax3.plot(reference_trajectory[:, 3], "bx-", label="Reference")

            ax4.set_title("d_steer (control input)")
            ax4.plot(controls, "rx-", label="MPC")
            ax4.plot(reference_control, "bx-", label="Reference")
            plt.show()

        # Publish error for debugging
        try:
            reference_position = trajectory.get_position_by_index(
                self.current_traj_index
            )
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
        except rospy.ROSException:
            pass
            # print("ROS Publishing Error")

        if self.TIME:
            print("Total time: ", 1000 * (time.time() - totaltime))
        return steer_angle


if __name__ == "__main__":
    # ModelPredictiveController.DEBUG = True
    ModelPredictiveController.PLOT = True
    mpc_controller = ModelPredictiveController()

    trajectory = Trajectory("/rb_ws/src/buggy/paths/quartermiletrack.json")

    # Offset initial pose slightly
    # initial_pose = Pose(
    #     # *trajectory.get_position_by_distance(10), trajectory.get_heading_by_distance(10)
    #     *trajectory.get_position_by_index(20), trajectory.get_heading_by_index(20)
    # )
    # initial_pose.x -= 3
    # initial_pose.y += 4
    # initial_pose.theta += 0.1
    # print(initial_pose)
    initial_pose = Pose(x=707.5893698488362, y=540, theta=-0.0292735217760258)

    mpc_controller.compute_control(initial_pose, trajectory, 5)
    mpc_controller.compute_control(initial_pose, trajectory, 5)
    ModelPredictiveController.PLOT = False

    for _ in range(100):
        mpc_controller.compute_control(initial_pose, trajectory, 5)
