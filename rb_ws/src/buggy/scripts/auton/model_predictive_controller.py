#!/usr/bin/env python3

import time
from threading import Lock

from numba import njit

import numpy as np
import osqp
from scipy import sparse

import rospy
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry


from pose import Pose
from trajectory import Trajectory
from controller import Controller
from world import World


import matplotlib.pyplot as plt


class ModelPredictiveController(Controller):
    """
    Convex Model Predictive Controller (MPC)
    """

    DEBUG = True
    PLOT = False
    TIME = False
    ROS = True

    # TODO: constants class
    # MPC Params
    WHEELBASE = 1.17 #m
    MASS = 62 #kg
    MAX_LAT_FORCE = 350 #N
    MAX_STEER = np.deg2rad(20) #degrees
    MIN_SPEED = 1.0
    MPC_TIMESTEP = 0.02
    MPC_HORIZON = 125
    LOOKAHEAD_TIME = 0.1  # seconds
    N_STATES = 4
    N_CONTROLS = 1

    # MPC Cost Weights
    state_cost = np.array([0.0001, 250, 5, 2500])  # x, y, theta, steer
    control_cost = np.array([5])  # d_steer
    final_state_cost = 2 * np.array([0.0001, 250, 5, 25])  # x, y, theta, steer

    # Control constraints
    # control_lb = np.array([-np.pi / 2])  # d_steer
    # control_ub = np.array([np.pi / 2])  # d_steer
    control_lb = np.array([-np.inf])  # d_steer
    control_ub = np.array([np.inf])  # d_steer

    # Solver params
    solver_settings: dict = {
        "verbose": DEBUG,
        "eps_abs": 1e-4,
        "eps_rel": 1e-4,
        "polish": 1,
        "time_limit": 1.5e-2,
        "warm_start": True,
        # "linsys_solver": "mkl pardiso",
    }

    # Precomputed arrays
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
    state_cost_diag = np.diag(state_cost)
    control_cost_diag = np.diag(control_cost)

    def __init__(self, buggy_name : str, start_index=0, ref_trajectory=None, ROS=False) -> None:
        # instantiate parent
        super(ModelPredictiveController, self).__init__(start_index, buggy_name)

        # Internal variables
        self.current_traj_index = 0  # Where in the trajectory we are currently
        self.current_speed = 0  # Speed of the buggy used for the MPC calculation. Assumed to be constant for now

        # Optimizer variables
        self.solver: osqp.OSQP = None

        self.debug_reference_pos_publisher = rospy.Publisher(
            buggy_name + "/auton/debug/reference_navsat", NavSatFix, queue_size=1
        )
        self.debug_error_publisher = rospy.Publisher(
            buggy_name + "/auton/debug/error", ROSPose, queue_size=1
        )

        self.solver = osqp.OSQP()

        self.C = sparse.kron(
            np.eye(self.MPC_HORIZON),
            np.hstack((self.control_jacobian(), -np.eye(self.N_STATES))),
            format="csc",
        )

        self.ROS = ROS
        self.lock = Lock()
        self.ref_trajectory = ref_trajectory

        if self.ROS:
            rospy.Subscriber('mpc/speed', Float64, self.update_speed)
            rospy.Subscriber('mpc/current_pose', Pose2D, self.update_trajectory)
            self.trajectory_publisher = rospy.Publisher('mpc/sol_trajectory',
                                                        Float64MultiArray,
                                                        queue_size=1)
            self.status_publisher = rospy.Publisher('mpc/solver_status',
                                                    Bool,
                                                    queue_size=1)
            rospy.init_node('mpc_calculator')

    def update_speed(self, msg):
        with self.lock:
            self.current_speed = msg.data

    def update_trajectory(self, msg):
        self.status_publisher.publish(True)

        pose = Pose(msg.x, msg.y, msg.theta)
        cur_time_s = rospy.get_time()
        t = np.linspace(cur_time_s + self.MPC_TIMESTEP,
                        cur_time_s + (self.MPC_HORIZON - 1) * self.MPC_TIMESTEP,
                        num=self.MPC_HORIZON)
        t = t.reshape((self.MPC_HORIZON, 1))
        sol_trajectory = self.compute_trajectory(pose, self.ref_trajectory, self.current_speed)

        timed_traj = np.hstack([t, sol_trajectory], dtype=float)

        n_msg = Float64MultiArray()
        # n_msg.data = timed_traj
        n_msg.data = np.reshape(timed_traj, (1 + self.N_STATES) * self.MPC_HORIZON)
        n_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        n_msg.layout.dim[0].label = "timestep"
        n_msg.layout.dim[0].size = self.MPC_HORIZON
        n_msg.layout.dim[0].stride = (1 + self.N_STATES) * self.MPC_HORIZON
        n_msg.layout.dim[1].label = "time, states"
        n_msg.layout.dim[1].size = (1 + self.N_STATES)
        n_msg.layout.dim[1].stride = (1 + self.N_STATES)

        self.trajectory_publisher.publish(n_msg)

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

    @staticmethod
    @njit
    def _state_jacobian_jit(state, speed, wheelbase, mpc_timestep):
        theta = state[2]
        steer = state[3]

        return np.array(
            [
                1,
                0,
                -speed * np.sin(theta) * mpc_timestep,
                0,
                0,
                1,
                speed * np.cos(theta) * mpc_timestep,
                0,
                0,
                0,
                1,
                speed / wheelbase * (1 / np.cos(steer) ** 2) * mpc_timestep,
                0,
                0,
                0,
                1,
            ],
            dtype=np.float32,
        ).reshape((4, 4))

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
        return self._state_jacobian_jit(
            state, self.current_speed, self.WHEELBASE, self.MPC_TIMESTEP
        )

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

    @staticmethod
    @njit
    def _rotate_state_cost_jit(cost, theta):
        return np.abs(
            np.array(
                [
                    np.cos(theta) * cost[0] - np.sin(theta) * cost[1],
                    np.sin(theta) * cost[0] + np.cos(theta) * cost[1],
                    cost[2],
                    cost[3],
                ]
            )
        )

    def rotate_state_cost(self, theta):
        """Return the state cost, but rotate the x and y costs by an inputted angle.
        This will allow the cost to be in the frame of the reference trajectory and not the
        global frame.

        Args:
            theta (Number): angle to rotate the cost by in radians
        """
        return self._rotate_state_cost_jit(self.state_cost, theta)

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

    @staticmethod
    @njit
    def transform_trajectory(trajectory, transform_matrix):
        """Transform a trajectory by a translation and rotation

        Args:
            trajectory (np.ndarray(Nx4)): trajectory to transform
            transform_matrix (np.ndarray): 3x3 transformation matrix
        """
        traj_homogeneous = np.vstack(
            (trajectory[:, 0:2].T, np.ones((1, trajectory.shape[0])))
        )
        traj_transformed = transform_matrix @ traj_homogeneous

        new_thetas = trajectory[:, 2] + np.arctan2(
            transform_matrix[1, 0], transform_matrix[0, 0]
        )
        new_thetas = np.arctan2(np.sin(new_thetas), np.cos(new_thetas))

        return np.stack(
            (
                traj_transformed[0, :].ravel(),
                traj_transformed[1, :].ravel(),
                new_thetas.ravel(),
                trajectory[:, 3].ravel(),
            ),
            -1,
        )

    first_iteration = True

    def compute_steering_limit(self, speed):
        # in radians
        max_steer = np.arctan(self.WHEELBASE * self.MAX_LAT_FORCE / (self.MASS * speed ** 2))
        return max_steer

    def compute_trajectory(self, current_pose: Pose, trajectory: Trajectory, current_speed: float):
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

        # Clamp speed to a minimum value
        current_speed = max(current_speed, self.MIN_SPEED)

        # Get reference pose
        if self.current_traj_index >= trajectory.get_num_points() - 1:
            return 0

        if self.TIME:
            t = time.time()
        self.current_speed = current_speed

        traj_index = trajectory.get_closest_index_on_path(
            current_pose.x,
            current_pose.y,
            start_index=self.current_traj_index -20,
            end_index=self.current_traj_index + 50,
            subsample_resolution=1000,
        )
        self.current_traj_index = max(traj_index, self.current_traj_index)
        traj_distance = trajectory.get_distance_from_index(self.current_traj_index)
        traj_distance += self.LOOKAHEAD_TIME * self.current_speed

        if self.DEBUG:
            print("Current traj index: ", self.current_traj_index)

        if self.current_traj_index >= trajectory.get_num_points() - 1:
            raise Exception("[MPC]: Ran out of path to follow!")

        # Get reference trajectory from the current traj index for the next [MPC_HORIZON] seconds
        knot_point_distances = np.linspace(
            traj_distance,
            traj_distance + self.current_speed * self.MPC_TIMESTEP * self.MPC_HORIZON,
            self.MPC_HORIZON,
        )
        knot_point_indices = trajectory.get_index_from_distance(knot_point_distances)
        reference_trajectory = trajectory.get_dynamics_by_index(
            knot_point_indices, self.WHEELBASE
        )
        reference_control = np.gradient(reference_trajectory[:, 3]) / self.MPC_TIMESTEP

        if self.TIME:
            ref_time = 1000.0 * (time.time() - t)

        if self.TIME:
            t = time.time()

        # Rotate reference trajectory to the frame of the current pose
        inverted_pose = current_pose.invert()
        reference_trajectory = self.transform_trajectory(
            reference_trajectory, inverted_pose.to_mat()
        )

        if self.TIME:
            ref_transform_time = 1000.0 * (time.time() - t)

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
        #
        # D = [C; X; U]
        # X selects all the states from z
        # U selects all the controls from z

        # Minimize 0.5 z^T H z such that lb <= D z <= ub
        if self.TIME:
            t = time.time()

        # H = sparse.block_diag(
        #     (
        #         self.control_cost_diag,
        #         *[
        #             m
        #             for k in range(0, self.MPC_HORIZON - 1)
        #             for m in [
        #                 np.diag(self.rotate_state_cost(reference_trajectory[k, 2])),
        #                 self.control_cost_diag,
        #             ]
        #         ],
        #         np.diag(
        #             self.rotate_final_state_cost(
        #                 reference_trajectory[self.MPC_HORIZON - 1, 2]
        #             )
        #         ),
        #     ),
        #     format="csc",
        # )
        H = sparse.diags(
            np.concatenate(
                [
                    self.control_cost,
                    *[
                        m
                        for k in range(0, self.MPC_HORIZON - 1)
                        for m in [
                            self.rotate_state_cost(reference_trajectory[k, 2]),
                            self.control_cost,
                        ]
                    ],
                    self.rotate_final_state_cost(
                        reference_trajectory[self.MPC_HORIZON - 1, 2]
                    ),
                ]
            ),
            format="csc",
        )
        if self.TIME:
            create_mat_time_H = 1000.0 * (time.time() - t)

        if self.TIME:
            t = time.time()

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

        # halfplane constraint
        # c = [n 0 0], where n is the normal vector of the halfplane in x-y space
        # p is the position of NAND in x-y space

        # n = np.array([100, 100])
        # p = np.array([0, 1])
        # c = np.concatenate((n, np.zeros((2, )))).reshape(1, self.N_STATES)

        # C2 = sparse.kron(
        #     np.eye(self.MPC_HORIZON),
        #     np.hstack(
        #         (
        #             np.zeros((1, self.N_CONTROLS)),
        #             c
        #         )
        #     ),
        #     format="csc",
        # )

        D = sparse.vstack([self.C + C1, self.X, self.U])
        # D = sparse.vstack([self.C + C1, self.X, self.U, C2])

        if self.TIME:
            create_mat_time_D = 1000.0 * (time.time() - t)

        if self.TIME:
            t = time.time()

        steer_limit = min(self.compute_steering_limit(current_speed), self.MAX_STEER)
        if self.DEBUG:
            print("max steer:", np.rad2deg(steer_limit))

        # State constraints
        state_lb = np.array([-np.inf, -np.inf, -np.inf, -steer_limit])  # x, y, theta, steer
        state_ub = np.array([np.inf, np.inf, np.inf, steer_limit])  # x, y, theta, steer

        lb = np.hstack(
            (
                -self.state_jacobian(reference_trajectory[0, :]) @ (self.state(0, 0, 0, 0) - reference_trajectory[0, :]),
                np.zeros(self.N_STATES * (self.MPC_HORIZON - 1)),
                np.tile(state_lb, self.MPC_HORIZON) - reference_trajectory.ravel(),
                np.tile(self.control_lb, self.MPC_HORIZON) - reference_control.ravel(),
                # np.tile(n.T @ p, self.MPC_HORIZON),
            )
        )
        ub = np.hstack(
            (
                -self.state_jacobian(reference_trajectory[0, :]) @ (self.state(0, 0, 0, 0) - reference_trajectory[0, :]),
                np.zeros(self.N_STATES * (self.MPC_HORIZON - 1)),
                np.tile(state_ub, self.MPC_HORIZON) - reference_trajectory.ravel(),
                np.tile(self.control_ub, self.MPC_HORIZON) - reference_control.ravel(),
                # np.tile(np.inf, self.MPC_HORIZON),
            )
        )

        if self.TIME:
            create_mat_time_bounds = 1000.0 * (time.time() - t)

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
            setup_time = 1000 * (time.time() - t)

        if self.TIME:
            t = time.time()
        results = self.solver.solve()

        steer_angle = results.x[self.N_CONTROLS + self.N_STATES - 1]
        solution_trajectory = np.reshape(results.x, (self.MPC_HORIZON, self.N_STATES + self.N_CONTROLS))
        state_trajectory = solution_trajectory[:, self.N_CONTROLS:(self.N_CONTROLS + self.N_STATES)]

        if not (results.info.status == "solved" or results.info.status == "solved inaccurate"):
            print("unable to solve!")
            return reference_trajectory

        state_trajectory += reference_trajectory

        if self.TIME:
            solve_time = 1000 * (time.time() - t)

        if self.TIME:
            t = time.time()

        if steer_angle == None:
            print("No solution found")

        if self.DEBUG:
            print("Control: ", steer_angle)

        if self.PLOT:
            # Plot the results
            _, [[ax1, ax2], [ax3, ax4]] = plt.subplots(2, 2)

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
            reference_position = trajectory.get_position_by_distance(
                traj_distance
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
            plot_time = 1000 * (time.time() - t)

        if self.TIME:
            print(" Ref Traj: ", ref_time)
            print(" Transform: ", ref_transform_time)
            print(" Create H: ", create_mat_time_H)
            print(" Create D: ", create_mat_time_D)
            print(" Create bounds: ", create_mat_time_bounds)
            print(" Setup: ", setup_time)
            print(" Solve: ", solve_time)
            print(" Plot/pub: ", plot_time)
            print("Total time: ", 1000 * (time.time() - totaltime))

        return state_trajectory
        # return steer_angle

    def compute_control(self,
        state_msg: Odometry, trajectory: Trajectory):

        current_rospose = state_msg.pose.pose
        current_pose = World.gps_to_world_pose(Pose.rospose_to_pose(current_rospose))
        current_speed = np.sqrt(
            state_msg.twist.twist.linear.x**2 + state_msg.twist.twist.linear.y**2
        )

        state_trajectory = self.compute_trajectory(current_pose, trajectory, current_speed)
        steer_angle = state_trajectory[0, self.N_STATES - 1]

        return steer_angle


if __name__ == "__main__":
    mpc_controller = ModelPredictiveController(
        ref_trajectory=Trajectory("/rb_ws/src/buggy/paths/quartermiletrack.json"),
        ROS=True)

    while not rospy.is_shutdown():
        rospy.spin()
    # # ModelPredictiveController.DEBUG = True
    # ModelPredictiveController.PLOT = True
    # mpc_controller = ModelPredictiveController()
    #
    # trajectory = Trajectory("/rb_ws/src/buggy/paths/quartermiletrack.json")
    #
    # # Offset initial pose slightly
    # # initial_pose = Pose(
    # #     # *trajectory.get_position_by_distance(10), trajectory.get_heading_by_distance(10)
    # #     *trajectory.get_position_by_index(20), trajectory.get_heading_by_index(20)
    # # )
    # # initial_pose.x -= 3
    # # initial_pose.y += 4
    # # initial_pose.theta += 0.1
    # # print(initial_pose)
    # initial_pose = Pose(x=707.5893698488362, y=540, theta=-0.0292735217760258)
    #
    # mpc_controller.compute_control(initial_pose, trajectory, 5)
    # mpc_controller.compute_control(initial_pose, trajectory, 5)
    # ModelPredictiveController.PLOT = False
    #
    # for _ in range(100):
    #     mpc_controller.compute_control(initial_pose, trajectory, 5)
