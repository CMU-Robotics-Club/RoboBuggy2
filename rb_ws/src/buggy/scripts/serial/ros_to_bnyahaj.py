#!/usr/bin/env python3

from host_comm import *
from threading import Lock

import threading

import rospy
import argparse
from geometry_msgs.msg import PoseWithCovariance

#Ros Message Imports
from std_msgs.msg import Float32, Float64, Bool, Int8
from nav_msgs.msg import Odometry as ROSOdom

import sys
sys.path.append("/rb_ws/src/buggy/scripts/auton")
from world import World

class Translator:
    def __init__(self, self_name, other_name):
        self.comms = Comms("/dev/ttyUSB0")
        self.steer_angle = 0
        self.lock = Lock()

        rospy.Subscriber(self_name + "/buggy/input/steering", Float64, self.set_steering)
        self.odom_publisher = rospy.Publisher(other_name + "/nav/odom", ROSOdom, queue_size=1)
        self.steer_send_rate = rospy.Rate(100)
        self.read_rate = rospy.Rate(1000)

    #Steering Angle Updater
    def set_steering(self, msg):
        #print("Steering angle: " + str(msg.data))
        # print("SET STEERING: " + str(msg.data))
        with self.lock:
            self.steer_angle = msg.data

    def writer_thread(self):
        print('Starting packet reading!')
        while True:
            with self.lock:
                self.comms.send_steering(self.steer_angle)
            self.steer_send_rate.sleep()

    def reader_thread(self):
        print('Starting packet sending!')
        while True:
            packet = self.comms.read_packet()
            if packet is not None:
                print("read odom")
                #Publish to odom topic x and y coord
                odom = ROSOdom()
                # convert to long lat
                lat, long = World.utm_to_gps(packet.x, packet.y)
                odom.pose.pose.position.x = long
                odom.pose.pose.position.y = lat

                self.odom_publisher.publish(odom)

            # # for debug
            # odom = ROSOdom()
            # # convert to long lat
            # lat, long = World.utm_to_gps(589846, 4477580)
            # # lat, long = World.utm_to_gps(packet.x, packet.y)
            # odom.pose.pose.position.x = long
            # odom.pose.pose.position.y = lat

            # self.odom_publisher.publish(odom)


            self.read_rate.sleep()



    def loop(self):
        p1 = threading.Thread(target=self.writer_thread)
        p2 = threading.Thread(target=self.reader_thread)

        p1.start()
        p2.start()

        p1.join()
        p2.join()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--self_name", type=str, help="name of ego-buggy", required=True)
    parser.add_argument("--other_name", type=str, help="name of other buggy", required=True)
    args, _ = parser.parse_known_args()
    self_name = args.self_name
    other_name = args.other_name

    rospy.init_node("ros_bnyahaj")
    translate = Translator(self_name, other_name)
    translate.loop()



