#!/usr/bin/env python3
import numpy as np
from controller import Controller

class BrakeController:
    MAX_LATERAL_ACCEL = 0.9 # in "g" based on regular vehicle
    BRAKING_GAIN = 2.0
    def __init__(self, use_binary_braking = True):
        # NOTE: Add more stuff here to keep track of PID stuff once I and D are implemented.
        self.binary_braking = use_binary_braking

    @staticmethod
    def calculate_lateral_accel(linear_speed: float, steering_angle: float) -> float:
        """Calculate the lateral acceleration given speed and steering angle for the CENTER of the
        buggy.

        Args:
            linear_speed (float): m/s
            steering_angle (float): deg

        Returns:
            float: lateral accel in "g"
        """
        if (steering_angle == 0.0):
            return 0.0
        radius_front_wheel = Controller.WHEELBASE / np.sin(np.deg2rad(steering_angle))
        radius_rear_wheel = Controller.WHEELBASE / np.tan(np.deg2rad(steering_angle))

        lat_accel_front_wheel = np.absolute((linear_speed**2)/radius_front_wheel) / 9.81
        lat_accel_rear_wheel = np.absolute((linear_speed**2)/radius_rear_wheel) / 9.81

        lat_accel = (lat_accel_front_wheel + lat_accel_rear_wheel) / 2

        return lat_accel

    def compute_braking(self, speed: float, steering_angle: float) -> float:
        """Wrapper for the type of braking controller we're using

        Args:
            speed (float): m/s
            steering_angle (float): degrees

        Returns:
            float: 0-1 (1 = full brake)
        """
        if self.binary_braking:
            return self._compute_binary_braking(speed, steering_angle)
        else:
            return self._compute_modulated_braking(speed, steering_angle)

    def _compute_binary_braking(self, speed: float, steering_angle: float) -> float:
        """Binary braking - using lateral acceleration

        Args:
            speed (float): m/s
            steering_angle (float): degrees

        Returns:
            float: 1 is brake, 0 is release
        """
        lat_accel = BrakeController.calculate_lateral_accel(speed, steering_angle)
        if (lat_accel > BrakeController.MAX_LATERAL_ACCEL):
            return 1.0
        else:
            return 0.0

    def _compute_modulated_braking(self, speed: float, steering_angle: float) -> float:
        """Using P controller for braking based on lateral acceleration of buggy. Modulated values
        from 0-1

        Args:
            speed (float): _description_
            steering_angle (float): _description_

        Returns:
            float: _description_
        """
        lat_accel = BrakeController.calculate_lateral_accel(speed, steering_angle)
        if (lat_accel < BrakeController.MAX_LATERAL_ACCEL):
            return 0.0
        else:
            error = lat_accel - BrakeController.MAX_LATERAL_ACCEL
            brake_cmd = 1.0 * error * BrakeController.BRAKING_GAIN
            brake_cmd = np.clip(brake_cmd, 0, 1)
            return brake_cmd

if __name__ == "__main__":
    brake_controller = BrakeController()
    speed = 15
    steering_angle = 4
    print("Lateral Accel: ", BrakeController.calculate_lateral_accel(speed, steering_angle))
    brake_cmd = brake_controller.compute_braking(speed, steering_angle)
    print("Braking CMD: ", brake_cmd)