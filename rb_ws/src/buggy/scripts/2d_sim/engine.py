#! /usr/bin/env python3

import re
import sys
import threading
import rospy
from geometry_msgs.msg import Pose, Twist, PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import numpy as np
import utm
sys.path.append("/rb_ws/src/buggy/scripts/auton")
from trajectory import Trajectory
from world import World


class Simulator:
    UTM_EAST_ZERO = 589702.87
    UTM_NORTH_ZERO = 4477172.947
    UTM_ZONE_NUM = 17
    UTM_ZONE_LETTER = "T"
    WHEELBASE = 1.17
    # Start positions for Outdoor track
    START_LAT = 40.443024364623916
    START_LONG = -79.9409643423245
    NOISE = True # Noisy outputs for nav/odom?

    def __init__(self, start_pos: str, velocity: float, buggy_name: str):
        """
        Args:
            heading (float): degrees start heading of buggy
        """
        # for X11 matplotlib (direction included)
        self.plot_publisher = rospy.Publisher(buggy_name + "/sim_2d/utm", Pose, queue_size=1)

        # simulate the INS's outputs (noise included)
        self.pose_publisher = rospy.Publisher(buggy_name + "/nav/odom", Odometry, queue_size=1)

        self.steering_subscriber = rospy.Subscriber(
            buggy_name + "/buggy/input/steering", Float64, self.update_steering_angle
        )
        # To read from velocity
        self.velocity_subscriber = rospy.Subscriber(
            buggy_name + "/velocity", Float64, self.update_velocity
        )
        # to plot on Foxglove (no noise)
        self.navsatfix_publisher = rospy.Publisher(
            buggy_name + "/state/pose_navsat", NavSatFix, queue_size=1
        )

        if Simulator.NOISE:
            # to plot on Foxglove (with noise)
            self.navsatfix_noisy_publisher = rospy.Publisher(
                buggy_name + "/state/pose_navsat_noisy", NavSatFix, queue_size=1
            )

        # (UTM east, UTM north, HEADING(degs))
        self.starting_poses = {
            "Hill1_NAND": (Simulator.UTM_EAST_ZERO + 0, Simulator.UTM_NORTH_ZERO + 0, -110),
            "Hill1_SC": (Simulator.UTM_EAST_ZERO + 20, Simulator.UTM_NORTH_ZERO + 30, -110),
            "WESTINGHOUSE": (589647, 4477143, -150),
            "UC_TO_PURNELL": (589635, 4477468, 160),
            "UC": (589681, 4477457, 160),
            "TRACK_EAST_END": (589953, 4477465, 90),
            "TRACK_RESNICK": (589906, 4477437, -20),
            "GARAGE": (589846, 4477580, 180),
            "PASS_PT": (589491, 4477003, -160),
            "FREW_ST": (589646, 4477359, -20),
            "FREW_ST_PASS": (589644, 4477368, -20),
            "RACELINE_PASS": (589468.02, 4476993.07, -160),
        }

        # Start position for End of Hill 2
        # self.e_utm = Simulator.UTM_EAST_ZERO - 3
        # self.n_utm = Simulator.UTM_NORTH_ZERO - 10

        # Start position for Outdoor track
        # self.e_utm = Simulator.UTM_EAST_ZERO + 110
        # self.n_utm = Simulator.UTM_NORTH_ZERO + 296

        # Start positions for Outdoor track
        # utm_coords = utm.from_latlon(Simulator.START_LAT, Simulator.START_LONG)
        # self.e_utm = utm_coords[0]
        # self.n_utm = utm_coords[1]

        # Use start_pos as key in starting_poses dictionary
        if start_pos in self.starting_poses:
            init_pose = self.starting_poses[start_pos]
        else:
            # Use start_pos as float representing distance down track to start from
            try:
                start_pos = float(start_pos)
                trajectory = Trajectory("/rb_ws/src/buggy/paths/buggycourse_safe_1.json")

                init_world_coords = trajectory.get_position_by_distance(start_pos)
                init_heading = np.rad2deg(trajectory.get_heading_by_distance(start_pos)[0])
                init_x, init_y = tuple(World.world_to_utm_numpy(init_world_coords)[0])

            # Use start_pos as (e_utm, n_utm, heading) coordinates
            except ValueError:
                # Extract the three coordinates from start_pos
                matches = re.match(
                    r"^\(?(?P<utm_e>-?[\d\.]+), *(?P<utm_n>-?[\d\.]+), *(?P<heading>-?[\d\.]+)\)?$",
                    start_pos
                )
                if matches == None: raise ValueError("invalid start_pos for " + buggy_name)
                matches = matches.groupdict()

                init_x = float(matches["utm_e"])
                init_y = float(matches["utm_n"])
                init_heading = float(matches["heading"])

            init_pose = init_x, init_y, init_heading

        self.e_utm, self.n_utm, self.heading = init_pose
        self.velocity = velocity # m/s

        self.steering_angle = 0  # degrees
        self.rate = 200  # Hz
        self.pub_skip = 2  # publish every pub_skip ticks

        self.lock = threading.Lock()

    def update_steering_angle(self, data: Float64):
        """Updates the steering angle as callback function for subscriber

        Args:
            data (Float64): angle in degrees
        """
        with self.lock:
            self.steering_angle = data.data

    def update_velocity(self, data: Float64):
        """Updates the velocity as callback function for subscriber

        Args:
            data (Float64): velocity in m/s
            source (string): whether incoming data is manual or simulated
        """
        with self.lock:
            self.velocity = data.data

    def get_steering_arc(self):
        # Adapted from simulator.py (Joseph Li)
        # calculate the radius of the steering arc
        #   1) If steering angle is 0, return infinity
        #   2) Otherwise, return radius of arc
        # If turning right, steering radius is positive
        # If turning left, steering radius is negative
        with self.lock:
            steering_angle = self.steering_angle
        if steering_angle == 0.0:
            return np.inf

        return Simulator.WHEELBASE / np.tan(np.deg2rad(steering_angle))

    def dynamics(self, state, v):
        """ Calculates continuous time bicycle dynamics as a function of state and velocity

        Args:
            state (np.Array): State vector [x, y, heading, steering] in m, m, rad, rad respectively
            v (Float 64): velocity in m/s

        Returns:
            dstate (np.Array): time derivative of state from dynamics
        """
        l = Simulator.WHEELBASE
        _, _, theta, delta = state

        return np.array([v * np.cos(theta),
                         v * np.sin(theta),
                         v / l * np.tan(delta),
                         0])

    def step(self):
        """Complete the step at a certain Hz here. Do physics"""
        with self.lock:
            heading = self.heading
            e_utm = self.e_utm
            n_utm = self.n_utm
            velocity = self.velocity
            steering_angle = self.steering_angle

        # RK4 to calculate next position
        h = 1/self.rate
        state = np.array([e_utm, n_utm, np.deg2rad(heading), np.deg2rad(steering_angle)])
        k1 = self.dynamics(state, velocity)
        k2 = self.dynamics(state + h/2 * k1, velocity)
        k3 = self.dynamics(state + h/2 * k2, velocity)
        k4 = self.dynamics(state + h/2 * k3, velocity)

        final_state = state + h/6 * (k1 + 2 * k2 + 2 * k3 + k4)

        e_utm_new, n_utm_new, heading_new, _ = final_state
        heading_new = np.rad2deg(heading_new)

        with self.lock:
            self.e_utm = e_utm_new
            self.n_utm = n_utm_new
            self.heading = heading_new

    def publish(self):
        """Publishes the pose the arrow in visualizer should be at"""
        p = Pose()
        time_stamp = rospy.Time.now()
        with self.lock:
            p.position.x = self.e_utm
            p.position.y = self.n_utm
            p.position.z = self.heading
            velocity = self.velocity

        self.plot_publisher.publish(p)  # publish to X11 matplotlib window

        # NavSatFix for usage with X11 matplotlib AND Foxglove plotting
        (lat, long) = utm.to_latlon(
            p.position.x,
            p.position.y,
            Simulator.UTM_ZONE_NUM,
            Simulator.UTM_ZONE_LETTER,
        )

        nsf = NavSatFix()
        nsf.latitude = lat
        nsf.longitude = long
        nsf.altitude = 260  # constant
        nsf.header.stamp = time_stamp
        self.navsatfix_publisher.publish(nsf)

        # Possibly Noisy Data
        lat_noisy = lat
        long_noisy = long
        velocity_noisy = velocity
        heading = np.deg2rad(p.position.z)
        heading_noisy = heading

        if (Simulator.NOISE):
            lat_noisy = lat + np.random.normal(0, 1e-6)  # ~10cm error
            long_noisy = long + np.random.normal(0, 1e-6)  # ~10cm error
            velocity_noisy = velocity + np.random.normal(0, 0.01)
            heading_noisy = heading + np.random.normal(0, 0.01)

            # Publish a new point on Foxglove to indicate the noisy location
            nsf_noisy = NavSatFix()
            nsf_noisy.latitude = lat_noisy
            nsf_noisy.longitude = long_noisy
            nsf_noisy.header.stamp = time_stamp
            self.navsatfix_noisy_publisher.publish(nsf_noisy)

        # Odometry for using with autonomous code
        odom = Odometry()
        odom.header.stamp = time_stamp

        odom_pose = Pose()
        odom_pose.position.x = long_noisy  # may not be noisy depending on Simulator.NOISE flag
        odom_pose.position.y = lat_noisy
        odom_pose.position.z = 260

        odom_pose.orientation.x = 0
        odom_pose.orientation.y = 0

        odom_pose.orientation.z = np.sin(heading_noisy / 2)  # radians
        odom_pose.orientation.w = np.cos(heading_noisy / 2)

        odom_twist = Twist()
        odom_twist.linear.x = velocity_noisy  # may not be noisy depending on Simulator.NOISE flag

        odom.pose = PoseWithCovariance(pose=odom_pose)
        odom.twist = TwistWithCovariance(twist=odom_twist)

        # This is just dummy data
        odom.pose.covariance = [
            0.01,
            0,
            0,
            0,
            0,
            0,
            0,
            0.01,
            0,
            0,
            0,
            0,
            0,
            0,
            0.01,
            0,
            0,
            0,
            0,
            0,
            0,
            0.01,
            0,
            0,
            0,
            0,
            0,
            0,
            0.01,
            0,
            0,
            0,
            0,
            0,
            0,
            0.01,
        ]

        self.pose_publisher.publish(odom)

    def loop(self):
        """Loop for the main simulator engine"""
        rate = rospy.Rate(self.rate)
        pub_tick_count = 0

        while not rospy.is_shutdown():
            self.step()

            # Publish every self.pub_skip ticks
            if pub_tick_count == self.pub_skip:
                self.publish()
                pub_tick_count = 0
            else:
                pub_tick_count += 1

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("sim_2d_engine")
    print("sim 2d eng args:")
    print(sys.argv)

    start_pos = sys.argv[1]
    velocity = float(sys.argv[2])
    buggy_name = sys.argv[3]

    sim = Simulator(start_pos, velocity, buggy_name)
    for i in range(100):
        sim.publish()
        rospy.sleep(0.01)

    # publish initial position, then sleep
    # so that auton stack has time to initialize
    # before buggy moves
    print("SIM PUBLISHED", rospy.get_time())
    if buggy_name == "SC":
        rospy.sleep(15.0)
    print("SIM STARTED STEPPING", rospy.get_time())
    sim.loop()

