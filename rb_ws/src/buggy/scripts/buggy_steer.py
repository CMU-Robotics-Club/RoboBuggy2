#!/usr/bin/python3
# # Sends commands to buggy
import rospy
from std_msgs.msg import Float64

INITIAL_STEERING_ANGLE = 0.0

steering_publisher = rospy.Publisher("SteeringOut_T", Float64, queue_size=10)
message = Float64(INITIAL_STEERING_ANGLE)
# steering_publisher.publish(message)

def set_steering_angle(angle):
    message = Float64(angle)
    steering_publisher.publish(message)

