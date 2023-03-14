#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Twist, PoseWithCovariance, TwistWithCovariance
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import threading
import numpy as np
import utm

from utils import Buggy


class FoxgloveSimulator:
    START_LAT = 40.441786291658254
    START_LON = -79.94158389000677

    def __init__(self):
        self.pose_publisher = rospy.Publisher("nav/odom", Odometry, queue_size=1)
        self.navsat_publisher = rospy.Publisher(
            "state/pose_navsat", NavSatFix, queue_size=1
        )

        rospy.Subscriber("buggy/input/steering", Float64, self.update_steering_angle)

        self.lat = self.START_LAT
        self.lon = self.START_LON
        self.heading = -np.pi / 2  # radians
        self.velocity = 15  # m/s

        self.steering_angle = 0  # degrees

        self.rate = 50  # Hz

        self.lock = threading.Lock()

    def update_steering_angle(self, data: Float64):
        """Updates the steering angle as callback function for subscriber

        Args:
            data (Float64): angle in degrees
        """
        with self.lock:
            self.steering_angle = data.data

    def step(self):
        """Complete the step at a certain Hz here. Do physics"""
        with self.lock:
            heading = self.heading
            lat = self.lat
            lon = self.lon
            velocity = self.velocity
            steering_angle = self.steering_angle

        utm_coords = utm.from_latlon(lat, lon)
        x = utm_coords[0]
        y = utm_coords[1]

        # Convert steering angle to radians from degrees
        steering_angle = np.deg2rad(steering_angle)

        # Bicycle model
        x_new = x + (velocity / self.rate) * np.cos(heading + steering_angle)
        y_new = y + (velocity / self.rate) * np.sin(heading + steering_angle)
        heading_new = (
            heading + (velocity / self.rate) * np.sin(steering_angle) / Buggy.WHEELBASE
        )

        lat_new, lon_new = utm.to_latlon(x_new, y_new, utm_coords[2], utm_coords[3])

        with self.lock:
            self.lat = lat_new
            self.lon = lon_new
            self.heading = heading_new

    def publish(self):
        """Publishes the pose the arrow in visualizer should be at"""

        p = Pose()
        p.position.x = self.lon
        p.position.y = self.lat
        p.position.z = 260

        p.orientation.x = 0
        p.orientation.y = 0
        p.orientation.z = np.sin(self.heading / 2)
        p.orientation.w = np.cos(self.heading / 2)

        t = Twist()
        t.linear.x = self.velocity

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.pose = PoseWithCovariance(pose=p)
        odom.twist = TwistWithCovariance(twist=t)

        self.pose_publisher.publish(odom)

        navsat = NavSatFix()
        navsat.header.stamp = rospy.Time.now()
        navsat.header.frame_id = "map"
        navsat.latitude = self.lat
        navsat.longitude = self.lon
        navsat.altitude = 260

        self.navsat_publisher.publish(navsat)

    def loop(self):
        """Loop for the main simulator engine"""
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.step()
            self.publish()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("sim_foxglove")
    sim = FoxgloveSimulator()
    sim.loop()
