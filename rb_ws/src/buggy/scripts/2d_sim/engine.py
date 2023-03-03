#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import threading
import numpy as np

class Simulator:
    UTM_EAST_ZERO = 589702.62
    UTM_NORTH_ZERO = 4477172.17
    def __init__(self, heading: float):
        """
        Args:
            heading (float): degrees start heading of buggy
        """
        self.plot_publisher = rospy.Publisher("sim_2d/utm", Pose, queue_size=10)
        self.steering_subscriber = rospy.Subscriber("sim_2d/steering", Float32, self.update_steering_angle)

        self.e_utm = Simulator.UTM_EAST_ZERO
        self.n_utm = Simulator.UTM_NORTH_ZERO
        self.heading = heading # degrees
        self.velocity = 10 # m/s

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
    
    def step(self):
        """Complete the step at a certain Hz here. Do physics
        """
        with self.lock:
            heading = self.heading
            e_utm = self.e_utm
            n_utm = self.n_utm
            velocity = self.velocity
            steering_angle = self.steering_angle
        
        e_utm_new = e_utm + (velocity/self.rate) * np.cos(np.deg2rad(heading))
        n_utm_new = n_utm + (velocity/self.rate) * np.sin(np.deg2rad(heading))

        heading_new = heading + (steering_angle/self.rate)

        with self.lock:
            self.e_utm = e_utm_new
            self.n_utm = n_utm_new
            self.heading = heading_new

    def publish(self):
        """Publishes the pose the arrow in visualizer should be at
        """
        p = Pose()
        with self.lock:
            p.position.x = self.e_utm
            p.position.y = self.n_utm
            p.position.z = self.heading
        self.plot_publisher.publish(p)
        
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
    sim = Simulator(-90)
    sim.loop()