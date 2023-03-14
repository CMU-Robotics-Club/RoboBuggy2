#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import threading
import numpy as np
import utm
import time

class Simulator:
    UTM_EAST_ZERO = 589702.87
    UTM_NORTH_ZERO = 4477172.947
    UTM_ZONE_NUM = 17
    UTM_ZONE_LETTER = "T"
    WHEELBASE = 1.17
    def __init__(self, heading: float):
        """
        Args:
            heading (float): degrees start heading of buggy
        """
        self.plot_publisher = rospy.Publisher("sim_2d/utm", Pose, queue_size=10)
        self.steering_subscriber = rospy.Subscriber("sim_2d/steering", Float32, self.update_steering_angle)
        self.velocity_subscriber = rospy.Subscriber("sim_2d/velocity", Float32, self.update_velocity)
        self.navsatfix_publisher = rospy.Publisher("sim_2d/navsatfix", NavSatFix, queue_size=10)

        self.e_utm = Simulator.UTM_EAST_ZERO + 15
        self.n_utm = Simulator.UTM_NORTH_ZERO - 10
        self.heading = heading # degrees
        self.velocity = 0 # m/s

        self.steering_angle = 0 # degrees

        self.rate = 10 # Hz

        self.lock = threading.Lock()
    
    def update_steering_angle(self, data: Float32):
        """Updates the steering angle as callback function for subscriber

        Args:
            data (Float32): angle in degrees
        """
        with self.lock:
            self.steering_angle = data.data
    
    def update_velocity(self, data: Float32):
        """Updates the velocity as callback function for subscriber

        Args:
            data (Float32): velocity in m/s
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
    
    def step(self):
        """Complete the step at a certain Hz here. Do physics
        """
        with self.lock:
            heading = self.heading
            e_utm = self.e_utm
            n_utm = self.n_utm
            velocity = self.velocity
            steering_angle = self.steering_angle
    
        # Calculate new position
        if steering_angle == 0.0:
            # Straight
            e_utm_new = e_utm + (velocity/self.rate) * np.cos(np.deg2rad(heading))
            n_utm_new = n_utm + (velocity/self.rate) * np.sin(np.deg2rad(heading))
            heading_new = heading
        else:
            # steering radius
            radius = self.get_steering_arc()

            distance = (velocity/self.rate)

            delta_heading = distance/radius
            heading_new = heading + np.rad2deg(delta_heading)

            e_utm_new = e_utm + (velocity/self.rate) * np.cos(np.deg2rad(heading_new))
            n_utm_new = n_utm + (velocity/self.rate) * np.sin(np.deg2rad(heading_new))

        
        with self.lock:
            self.e_utm = e_utm_new
            self.n_utm = n_utm_new
            self.heading = heading_new

    def publish(self):
        """Publishes the pose the arrow in visualizer should be at
        """
        p = Pose()
        nsf = NavSatFix()
        with self.lock:
            p.position.x = self.e_utm
            p.position.y = self.n_utm
            p.position.z = self.heading
        
        (lat, long) = utm.to_latlon(p.position.x, p.position.y, Simulator.UTM_ZONE_NUM, Simulator.UTM_ZONE_LETTER)
        nsf.latitude = lat
        nsf.longitude = long
        nsf.header.stamp = rospy.Time.now()
        self.plot_publisher.publish(p)
        self.navsatfix_publisher.publish(nsf)

        
    def loop(self):
        """Loop for the main simulator engine
        """
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.step()
            self.publish()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("sim_2d_engine")
    sim = Simulator(-175)
    sim.loop()