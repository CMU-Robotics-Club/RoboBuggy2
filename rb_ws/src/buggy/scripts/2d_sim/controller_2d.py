#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import threading
import numpy as np

class Controller:
    def __init__(self):
        self.steering_publisher = rospy.Publisher("sim_2d/steering", Float64, queue_size=10)
        self.velocity_publisher = rospy.Publisher("buggy/velocity", Float64, queue_size=10)
        
        self.steering_angle = 0
        self.velocity = 0

        self.lock = threading.Lock()
        self.set_steering(0)
        self.set_velocity(0)
    
    def set_steering(self, angle: float):
        """Set the steering angle and publish to simulator engine

        Args:
            angle (float): degrees, + is CCW
        """
        msg = Float64()
        msg.data = angle

        with self.lock:
            self.steering_angle = angle
        
        self.steering_publisher.publish(msg)
    
    def set_velocity(self, vel: float):
        """Set the velocity and publish to simulator engine

        Args:
            vel (float): velocity (m/s)
        """
        msg = Float64()
        msg.data = vel

        with self.lock:
            self.velocity = vel
        
        self.velocity_publisher.publish(msg)

if __name__ == "__main__":
    rospy.init_node("sim_2d_controller")
    controller = Controller()
    rate = rospy.Rate(5)
    i = 0
    while not rospy.is_shutdown():
        controller.set_velocity(15)
        rate.sleep()

