#!/usr/bin/python3

import numpy as np
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String 

buggy_steer_publisher = rospy.Publisher("SteeringOut_T", Float64, queue_size=10)
buggy_brake_publisher = rospy.Publisher("BrakingOut_T", Float64, queue_size=10)
buggy_lights_publisher = rospy.Publisher("LightsOut_T", String, queue_size = 10)
rospy.init_node("buggy_commands", anonymous=True)

def set_steer_angle(angle: Float64): # angle is a float
    if not rospy.is_shutdown():
        angle_command = Float64(angle)
        buggy_steer_publisher.publish(angle_command)

def set_brake(brake: Float64): # brake if float 0-100
    if not rospy.is_shutdown():
        brake_command = Float64(brake)
        buggy_brake_publisher.publish(brake_command)

def set_light(light: String): # light is string (e.g. "Standby", "Error")
    if not rospy.is_shutdown():
        light_command = light
        buggy_lights_publisher.publish(light_command)