#!/usr/bin/python3
# Sends commands to buggy
import rospy
from std_msgs.msg import Float64

INITIAL_BRAKE = 100.0

braking_publisher = rospy.Publisher("BrakingOut_T", Float64, queue_size = 10)
message = Float64(INITIAL_BRAKE)
# braking_publisher.publish(message)
    

def set_brake(brake):
    message = Float64(brake)
    braking_publisher.publish(message)
