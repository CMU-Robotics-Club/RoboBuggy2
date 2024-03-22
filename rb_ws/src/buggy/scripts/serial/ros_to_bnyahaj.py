#!/usr/bin/env python3

import threading
from threading import Lock

import sys
sys.path.append("/rb_ws/src/buggy/scripts/auton")

import argparse
import rospy

#Ros Message Imports
from std_msgs.msg import Float64, Bool, Int8
from nav_msgs.msg import Odometry as ROSOdom

from host_comm import *

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

        self.rc_steering_angle_publisher = rospy.Publisher(self_name + "/buggy/debug/rc_steering_angle", Float64, queue_size=1)
        self.steering_angle_publisher = rospy.Publisher(self_name + "/buggy/debug/steering_angle", Float64, queue_size=1)
        self.battery_voltage_publisher = rospy.Publisher(self_name + "/buggy/debug/battery_voltage", Float64, queue_size=1)
        self.operator_ready_publisher = rospy.Publisher(self_name + "/buggy/debug/operator_ready", Bool, queue_size=1)
        self.steering_alarm_publisher = rospy.Publisher(self_name + "/buggy/debug/steering_alarm", Bool, queue_size=1)
        self.brake_status_publisher = rospy.Publisher(self_name + "/buggy/debug/brake_status", Bool, queue_size=1)
        self.use_auton_steer_publisher = rospy.Publisher(self_name + "/buggy/debug/use_auton_steer", Bool, queue_size=1)
        self.rc_uplink_qual_publisher = rospy.Publisher(self_name + "/buggy/debug/rc_uplink_quality", Int8, queue_size=1)
        self.nand_fix_publisher = rospy.Publisher(self_name + "/buggy/debug/nand_fix", Int8, queue_size=1)
        # self.padding_1_publisher = rospy.Publisher(self_name + "/buggy/debug/padding_1", Int8, queue_size=1)
        # self.padding_2_publisher = rospy.Publisher(self_name + "/buggy/debug/padding_2", Int8, queue_size=1)

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

            # print("trying to read odom")
            if isinstance(packet, Odometry):
                # print("packet", packet.x, packet.y)
                #Publish to odom topic x and y coord
                odom = ROSOdom()
                # convert to long lat
                lat, long = World.utm_to_gps(packet.y, packet.x)
                odom.pose.pose.position.x = long
                odom.pose.pose.position.y = lat

                self.odom_publisher.publish(odom)
            elif isinstance(packet, tuple): #Are there any other packet that is a tuple
                # print(packet)
                self.rc_steering_angle_publisher.publish(Float64(packet[0]))
                self.steering_angle_publisher.publish(Float64(packet[1]))
                self.battery_voltage_publisher.publish(Float64(packet[2]))
                self.operator_ready_publisher.publish(Bool(packet[3]))
                self.steering_alarm_publisher.publish(Bool(packet[4]))
                self.brake_status_publisher.publish(Bool(packet[5]))
                self.use_auton_steer_publisher.publish(Bool(packet[6]))
                # self.rc_uplink_qual_publisher.publish(Int8(packet[7]))
                # self.nand_fix_publisher.publish(Int8(packet[8]))
                # self.padding_1_publisher.publish(packet[9][0])
                # self.padding_2_publisher.publish(packet[9][1])


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



