#!/usr/bin/env python3

from host_comm import *
from threading import Lock

import rospy
import argparse
from geometry_msgs.msg import Pose

#Ros Message Imports
from std_msgs.msg import Float32, Float64, Bool, Int8
from nav_msgs.msg import Odometry as ros_odom


parser = argparse.ArgumentParser()
parser.add_argument("--self_name", type=str, help="name of ego-buggy", required=True)
parser.add_argument("--other_name", type=str, help="name of other buggy", required=True)
args, _ = parser.parse_known_args()
self_name = args.self_name
other_name = args.other_name

comms = Comms("/dev/ttyUSB0")

#Steering Angle Updater
def send_steering(msg):
    print("Steering angle: " + str(msg.data))
    comms.send_steering(msg.data)

rospy.Subscriber(self_name + "/buggy/input/steering", Float64, send_steering)

odom_publisher = rospy.Publisher(other_name + "/nav/odom", ros_odom, queue_size=1)

print('Starting packet reading!')
while True:
    packet = comms.read_packet()
    if packet is not None:
        print(packet)
        #Publish to odom topic x and y coord
        odom = ros_odom()
        odom_pose = Pose()
        odom_pose.position.x = packet.x
        odom_pose.position.y = packet.y

        odom.pos = odom_pose

        print(odom)
        odom_publisher.publish(odom)    