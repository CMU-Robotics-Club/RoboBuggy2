#!/usr/bin/env python3

import argparse

import rospy

from std_msgs.msg import Bool, Int8, Float64

class Watchdog:

    STEERING_DEVIANCE = 4 #deg

    def __init__(self, self_name) -> None:
        self.alarm = 0 #Check this alarm value
        """
        0 - OK
        1 - WARNING
        2 - ERROR        
        """

        self.commanded_steering = 0
        self.inAutonSteer = False

        rospy.Subscriber(
            self_name + "/buggy/input/steering", Float64, self.set_input_steering
        )
        rospy.Subscriber(
            self_name + "/buggy/debug/steering_angle", Float64, self.check_stepper_steering
        )
        rospy.Subscriber(
            self_name + "/buggy/debug/use_auton_steer", Bool, self.set_auton_steer
        )

        self.alarm_publisher = rospy.Publisher(self_name + "/debug/sanity_warning", Int8, queue_size=1)
        self.alarm_publish_rate = rospy.Rate(100) #10ms per alarm

    def set_input_steering(self, msg):
        rospy.logdebug("Got input steering of: "  + str(msg.data))
        self.commanded_steering = msg.data

    def set_auton_steer(self, msg):
        if (msg.data and not self.inAutonSteer):
            rospy.loginfo("ENTERED AUTON")
        if (not msg.data and self.inAutonSteer):
            rospy.logwarn("EXITED AUTON")
        self.inAutonSteer = msg.data

    def check_stepper_steering(self, msg):
        stepper_steer = msg.data
        rospy.logdebug("Firmware's reported stepper degree: " + str(stepper_steer))
        if abs(stepper_steer - self.commanded_steering) > Watchdog.STEERING_DEVIANCE:
            self.alarm = 2 # ERROR
            if self.inAutonSteer:
                rospy.logerr("STEPPER DEVIANCE (DEGREES OFF): " +  str(abs(stepper_steer - self.commanded_steering)))
            else:
                rospy.logdebug("(Non Auton) Stepper Deviance of: " + str(abs(stepper_steer - self.commanded_steering)))

        elif abs(stepper_steer - self.commanded_steering) > Watchdog.STEERING_DEVIANCE//2:
            if self.inAutonSteer:
                rospy.logwarn("STEPPER POSSIBILY DEVIATING (DEGREES OFF):  " + str(abs(stepper_steer - self.commanded_steering)))
                self.alarm = max(self.alarm, 1)
            else:
                rospy.logdebug("(Non Auton) Stepper possibly deviating: " + str(abs(stepper_steer - self.commanded_steering)))

    def loop(self):
        while True:
            #publish alarm
            ros_alarm = Int8()
            ros_alarm.data = self.alarm
            self.alarm_publisher.publish(ros_alarm)

            self.alarm_publish_rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--self_name", type=str, help="name of ego-buggy", required=True
    )
    args, _ = parser.parse_known_args()
    self_name = args.self_name
    rospy.init_node("watchdog")
    rospy.loginfo("INITIALIZED WATCHDOG NODE")
    watchdog = Watchdog(self_name=self_name)
    watchdog.loop()