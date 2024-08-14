#!/usr/bin/env python3

import sys
import numpy as np
import scipy
import scipy.linalg
import rospy
import time

from nav_msgs.msg import Odometry as ROSOdom

STATE_SPACE_DIM = 3
MEASUREMENT_SPACE_DIM = 2

class UKF:
    def __init__(self, wheelbase : float, zeroth_sigma_point_weight : float, process_noise : np.ndarray, gps_noise : np.ndarray):
        self.wheelbase = wheelbase
        self.zeroth_sigma_point_weight = zeroth_sigma_point_weight
        self.speed = 0
        self.process_noise = process_noise
        self.gps_noise = gps_noise

        self.predicted_state_est = np.zeros((STATE_SPACE_DIM, 1))
        self.predicted_state_cov = np.zeros((STATE_SPACE_DIM, STATE_SPACE_DIM))

        self.updated_state_est = np.zeros((STATE_SPACE_DIM, 1))
        self.updated_state_cov = np.zeros((STATE_SPACE_DIM, STATE_SPACE_DIM))

        self.last_time = time.time()

        
    
    def generate_sigmas(self, mean : np.ndarray, covariance : np.ndarray, sigmas : list[np.ndarray], weights : list[float]):
        A = scipy.linalg.sqrtm(covariance)

        weights[0] = self.zeroth_sigma_point_weight
        sigmas[0] = mean

        for i in range(STATE_SPACE_DIM):
            s = (STATE_SPACE_DIM / (1 - weights[0])) ** 0.5
            A_col = A[:, i]
            sigmas[i+1] = mean + A_col
            sigmas[i + 1 + STATE_SPACE_DIM] = mean - A_col
        
        for i in range(1, 2 * STATE_SPACE_DIM + 1):
            weights[i] = (1 - weights[0]) / (2 * STATE_SPACE_DIM)
    
    def dynamics(self, state : np.ndarray, steering : float ) -> np.ndarray:
        x = np.zeros((STATE_SPACE_DIM, 1), dtype=np.float64)
        x[0, 0] = self.speed * np.cos(state[2, 0])
        x[1, 0] = self.speed * np.sin(state[2, 0])
        x[2, 0] = self.speed * np.tan(steering) / self.wheelbase
        return x

    def rk4(self, state : np.ndarray, steering : float, dt : float) -> np.ndarray:
        k1 = self.dynamics(state, steering)
        k2 = self.dynamics(state + (k1 * (dt / 2)), steering)
        k3 = self.dynamics(state + (k2 * (dt / 2)), steering)
        k4 = self.dynamics(state + (k3 * dt), steering)

        return state + ((k1 + (k2 * 2.0) + (k3 * 2.0) + k4) * (dt / 6))

    def state_to_measurement(vector : np.ndarray) -> np.ndarray:
        m = np.zeros((MEASUREMENT_SPACE_DIM, 1))
        m[:, 0] = vector[:2, 0]
        return m

    def set_speed(self, speed):
        self.speed = speed

    def predict(self, curr_state_est : np.narray, curr_state_cov : np.ndarray, steering : float, dt : float):
        state_sigmas = [np.zeros((STATE_SPACE_DIM, 1))] * (2 * STATE_SPACE_DIM + 1)
        state_weights = [0.0] * (2 * STATE_SPACE_DIM + 1)
        self.generate_sigmas(curr_state_est, curr_state_cov, state_sigmas, state_weights)

        for i in range(2 * STATE_SPACE_DIM + 1):
            state_sigmas[i] = self.rk4(state_sigmas[i], steering, dt)

        self.predicted_state_est = np.zeros((STATE_SPACE_DIM, 1))
        self.predicted_state_cov = np.zeros((STATE_SPACE_DIM, STATE_SPACE_DIM))

        for i in range(2 * STATE_SPACE_DIM + 1):
            self.predicted_state_est += state_sigmas[i] * state_weights[i]
        
        for i in range(2 * STATE_SPACE_DIM + 1):
            m = state_sigmas[i] - self.predicted_state_est
            self.predicted_state_cov += ((m * m.T) * state_weights[i])
        
        self.predicted_state_cov += self.process_noise * dt

    def update(self, curr_state_est : np.narray, curr_state_cov : np.ndarray, measurement : np.ndarray):

        state_sigmas = [np.zeros((STATE_SPACE_DIM, 1))] * (2 * STATE_SPACE_DIM + 1)
        weights = [0.0] * (2 * STATE_SPACE_DIM + 1)
        self.generate_sigmas(curr_state_est, curr_state_cov, state_sigmas, weights)

        measurement_sigmas = [np.zeros((MEASUREMENT_SPACE_DIM, 1))] * (2 * STATE_SPACE_DIM + 1)
        for i in range(2 * STATE_SPACE_DIM + 1):
            measurement_sigmas[i] = self.state_to_measurement(state_sigmas[i])
        
        predicted_measurement = np.zeros((MEASUREMENT_SPACE_DIM, 1))
        for i in range(2 * STATE_SPACE_DIM + 1):
            predicted_measurement += measurement_sigmas[i] * weights[i]

        innovation_cov = np.zeros((MEASUREMENT_SPACE_DIM, MEASUREMENT_SPACE_DIM))
        cross_cov = np.zeros((STATE_SPACE_DIM, MEASUREMENT_SPACE_DIM))

        for i in range(2 * STATE_SPACE_DIM + 1):
            m = measurement_sigmas[i] - predicted_measurement
            innovation_cov += (m * m.T) * weights[i]
            cross_cov += ((state_sigmas[i] - curr_state_est) * m.T) * weights[i]

        innovation_cov += self.gps_noise
        kalman_gain = cross_cov @ innovation_cov

        print("Measurement: ", measurement)
        print("Predicted measurement:", predicted_measurement)
        print("Kalman gain:", kalman_gain)
        self.updated_state_est = curr_state_est + (kalman_gain * (measurement - predicted_measurement))
        self.updated_state_cov = curr_state_cov - (kalman_gain * (innovation_cov * kalman_gain.T))


