#! /usr/bin/env python3
import sys
import threading
import rospy
from std_msgs.msg import Float64

class Controller:
    def __init__(self, buggy_name):
        self.steering_publisher = rospy.Publisher(buggy_name + "/input/steering", Float64, queue_size=10)
        self.velocity_publisher = rospy.Publisher(buggy_name + "/velocity", Float64, queue_size=10)
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
    buggy_name = sys.argv[1]
    controller = Controller(buggy_name)
    rate = rospy.Rate(5)
    i = 0
    while not rospy.is_shutdown():
        rate.sleep()

