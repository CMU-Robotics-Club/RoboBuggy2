#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64

class VelocityUpdater:
    def __init__(self):
        self.vel_publisher = rospy.Publisher(
            "buggy/velocity", Float64, queue_size=1
        )
        self.velocity: float = 15.0
        self.accel: float = 0.1 # Dummy value for acceleration
        
        self.rate = 100

    def tick(self):
        new_velocity = Float64()
        new_velocity.data = self.velocity + self.accel / self.rate

        self.velocity = new_velocity.data
        self.vel_publisher.publish(new_velocity)

if __name__ == "__main__":
    vel = VelocityUpdater()

    while not rospy.is_shutdown():
        vel.tick()