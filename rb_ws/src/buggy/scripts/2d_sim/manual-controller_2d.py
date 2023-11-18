#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import threading
import numpy as np
from controller_2d import Controller

class ManualController (Controller):
    def __init__(self):
        self.manual_velocity_publisher = rospy.Publisher("buggy/man_velocity", Float64, queue_size=10)
        Controller.__init__(self)
        
    
    def set_steering(self, angle: float):
        Controller.set_steering(self, angle)
    
    def set_velocity(self, vel: float):  
        """Set the velocity and publish to simulator engine

        Args:
            vel (float): velocity (m/s)
        """
        msg = Float64()
        msg.data = vel
        with self.lock:
            self.velocity = vel
        self.manual_velocity_publisher.publish(msg) 

if __name__ == "__main__": 
    rospy.init_node("manual_sim_2d_controller")
    m_controller = ManualController()
    rate = rospy.Rate(5)
    i = 0
    while not rospy.is_shutdown():
        rate.sleep()
