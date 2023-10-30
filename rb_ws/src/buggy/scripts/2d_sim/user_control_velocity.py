#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import keyboard

class VelocityUpdater:
    def __init__(self):
        self.vel_publisher = rospy.Publisher(
            "buggy/velocity", Float64, queue_size=1
        )
        self.velocity: float = 0
        self.accel: float = 0.1 # Dummy value for acceleration
        
        self.rate = 100

    def tick(self):
        new_velocity = Float64()

        event = keyboard.read_event()
        if event.event_type == 'down':
            if event.name == 'w':
                new_velocity.data = self.velocity + 1
            if event.name == 's':
                new_velocity.data = self.velocity - 1

        # new_velocity.data = self.velocity + self.accel / self.rate

        self.velocity = new_velocity.data
        self.vel_publisher.publish(new_velocity)

if __name__ == "__main__":
    vel = VelocityUpdater()

    while not rospy.is_shutdown():
        vel.tick()