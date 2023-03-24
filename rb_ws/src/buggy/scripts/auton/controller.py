from abc import ABC, abstractmethod

import numpy as np
from trajectory import Trajectory
from pose import Pose


class Controller(ABC):
    """
    Base class for all controllers.

    The controller takes in the current state of the buggy and a reference
    trajectory. It must then compute the desired control output.

    The method that it does this by is up to the implementation, of course.
    Example schemes include Pure Pursuit, Stanley, and LQR.
    """

    WHEELBASE = 1.3

    @abstractmethod
    def compute_control(
        self, current_pose: Pose, trajectory: Trajectory, current_speed: float
    ):
        """
        Computes the desired control output given the current state and reference trajectory

        Args:
            state (numpy.ndarray [size: (3,)]): current pose (x, y, theta)
            trajectory (Trajectory): reference trajectory
            current_speed (float): current speed of the buggy

        Returns:
            float (desired steering angle)
        """
        raise NotImplementedError
