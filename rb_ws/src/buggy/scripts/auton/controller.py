from abc import ABC, abstractmethod

from trajectory import Trajectory
from pose import Pose
from sensor_msgs.msg import NavSatFix
from world import World
from nav_msgs.msg import Odometry


class Controller(ABC):
    """
    Base class for all controllers.

    The controller takes in the current state of the buggy and a reference
    trajectory. It must then compute the desired control output.

    The method that it does this by is up to the implementation, of course.
    Example schemes include Pure Pursuit, Stanley, and LQR.
    """

    # TODO: move this to a constants class
    NAND_WHEELBASE = 1.3
    SC_WHEELBASE = 1.104
    current_traj_index = 0

    def __init__(self, start_index: int, buggy_name: str) -> None:
        self.buggy_name = buggy_name
        if buggy_name.upper() == 'NAND':
            Controller.WHEELBASE = self.NAND_WHEELBASE
        else:
            Controller.WHEELBASE = self.SC_WHEELBASE

        self.current_traj_index = start_index

    @abstractmethod
    def compute_control(
        self, state_msg: Odometry, trajectory: Trajectory,
    ) -> float:
        """
        Computes the desired control output given the current state and reference trajectory

        Args:
            state: (Odometry): state of the buggy, including position, attitude and associated twists
            trajectory (Trajectory): reference trajectory
            current_speed (float): current speed of the buggy

        Returns:
            float (desired steering angle)
        """
        raise NotImplementedError

    def plot_trajectory(
        self, current_pose: Pose, trajectory: Trajectory, current_speed: float
    ):
        """
        Plots the next x and previous y points along the trajectory

        Args:
            current_pose (Pose): current pose (x, y, theta) (UTM coordinates)
            trajectory (Trajectory): reference trajectory
            current_speed (float): current speed of the buggy
        """

        # Compute distance along path from current position
        traj_dist = trajectory.get_distance_from_index(self.current_traj_index)

        # Plot forward projections
        for i in range(1, len(self.forward_publishers)+1):
            forward_position = trajectory.get_position_by_distance(traj_dist + i*current_speed)
            navsat = NavSatFix()
            forward_gps = World.world_to_gps(*forward_position)
            navsat.latitude = forward_gps[0]
            navsat.longitude = forward_gps[1]
            self.forward_publishers[i-1].publish(navsat)

        # Plot backward positions
        for i in range(1, len(self.backward_publishers)+1):
            backward_position = trajectory.get_position_by_distance(traj_dist - i*current_speed)
            navsat = NavSatFix()
            backward_gps = World.world_to_gps(*backward_position)
            navsat.latitude = backward_gps[0]
            navsat.longitude = backward_gps[1]
            self.backward_publishers[i-1].publish(navsat)

    # TODO: do we want logging methods to be required (update_speed, update_trajectory)


