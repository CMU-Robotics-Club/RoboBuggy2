#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import threading
import numpy as np

class Controller:
    def __init__(self):
        self.steering_publisher = rospy.Publisher("sim_2d/steering", Float32, queue_size=10)
        self.steering_angle = 0

        self.lock = threading.Lock()
    
    def set_steering(self, angle: float):
        """Set the steering angle and publish to simulator engine

        Args:
            angle (float): degrees, + is CCW
        """
        msg = Float32()
        msg.data = angle

        with self.lock:
            self.steering_angle = angle
        
        self.steering_publisher.publish(msg)

if __name__ == "__main__":
    rospy.init_node("sim_2d_controller")
    controller = Controller()
    rate = rospy.Rate(5)
    i = 0
    while not rospy.is_shutdown():
        controller.set_steering(0)
        rate.sleep()

