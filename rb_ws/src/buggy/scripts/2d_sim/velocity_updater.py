#! /usr/bin/env python3
import rospy
import sys
from controller_2d import Controller
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import threading
import math

class VelocityUpdater:
    RATE = 100
    # Bubbles for updating acceleration based on position
    # represented as 4-tuples: (x-pos, y-pos, radius, acceleration)
    CHECKPOINTS: 'list[tuple[float,float,float,float]]' = [
        (589701, 4477160, 20, 0.5)
    ]

    def __init__(self, init_vel: float, buggy_name: str):
        self.pose_subscriber = rospy.Subscriber(
            buggy_name + "/sim_2d/utm", Pose, self.update_position
        )

        self.buggy_vel = init_vel
        self.accel = 0.0

        self.position = Point()
        self.position.x = 0
        self.position.y = 0
        self.position.z = 0

        self.controller = Controller(buggy_name)

        self.lock = threading.Lock()
    
    def update_position(self, new_pose: Pose):
        '''Callback function to update internal position variable when new
        buggy position is published
        
        Args:
            new_pose (Pose): Pose object from topic
        '''
        with self.lock:
            self.position = new_pose.position

    def calculate_accel(self):
        '''Check if the position of the buggy is in any of the checkpoints set
        in self.CHECKPOINTS, and update acceleration of buggy accordingly
        '''
        for (x,y,r,a) in self.CHECKPOINTS:
            dist = math.sqrt((x - self.position.x)**2 + (y - self.position.y)**2)
            if dist < r:
                self.accel = a
                break

    def step(self):
        '''Update acceleration and velocity of buggy for one timestep'''
        self.calculate_accel()

        new_velocity = self.buggy_vel + self.accel / self.RATE
        self.buggy_vel = new_velocity
        self.controller.set_velocity(new_velocity)


if __name__ == "__main__":
    rospy.init_node("vel_updater")

    init_vel = float(sys.argv[1])
    buggy_name = sys.argv[2]
    vel = VelocityUpdater(init_vel, buggy_name)
    rate = rospy.Rate(vel.RATE)

    while not rospy.is_shutdown():
        vel.step()
        rate.sleep()
