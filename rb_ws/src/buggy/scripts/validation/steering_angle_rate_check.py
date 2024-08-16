# A script to log battery data to a .txt file

import csv
import rospy
from std_msgs.msg import (
    Float64,
    Bool,
)  # Callback function to print the subscribed data on the terminal

file = open("steering_data.txt", "w", newline="")
writer = csv.writer(file)
writer.writerow(["Time", "steering_angle", "steering_angle_rate"])


class SteeringAngleRateChecker:
    """
    This is a class that checks if the Steering Angle is valid, and if it passes the max value of 100 deg/s, we will raise a flag.
    """

    def __init__(self):
        self.last_time_stamp = rospy.Time.now()
        self.steering_data_rate = 0
        self.last_steering_data = 0
        # threshold value and steering angle r in degree per second
        # TODO: Outsource this constant in a constraints section
        # TODO: Where is the steering data rate number coming from?
        # TODO: This should be ran for both buggies, so should be based on supposed namespaces, and shoud have better topic names.
        self.threshold_steering_data_rate = 100
        self.velocity_publisher = rospy.Publisher(
            "SteeringAngleRate", Float64, queue_size=10
        )
        self.flag_publisher = rospy.Publisher(
            "SteeringAngleRateFlag", Bool, queue_size=10
        )
        rospy.Subscriber(
            "/SC/buggy/input/steering", Float64, self.callback_steering_angle
        )

    def callback_steering_angle(self, steering_data):
        """
        When receiving steering angle, will calcuate new velocity, and log the speed at which the steering is changing as a velocity.
        """
        current_time = rospy.Time.now()
        self.steering_data_rate = (
            steering_data.data - self.last_steering_data
        ) / rospy.Time.to_sec(current_time - self.last_time_stamp)
        self.steering_data_rate_flag = bool(
            (abs(self.steering_data_rate) > self.threshold_steering_data_rate)
        )
        self.last_time_stamp = current_time
        self.last_steering_data = steering_data.data
        self.steering_steering_data_rate_publisher()
        writer.writerow([current_time, steering_data.data, self.steering_data_rate])

    def steering_steering_data_rate_publisher(self):
        """
        Publishes into two seperate nodes, SteeringAngleRate, and SteeringAngleRateFlag
        """
        velocity_message = Float64()
        velocity_message.data = self.steering_data_rate
        self.velocity_publisher.publish(velocity_message)

        flag_message = Bool()
        flag_message.data = self.steering_data_rate_flag
        self.flag_publisher.publish(flag_message)


if __name__ == "__main__":
    # TODO: Implement this as a validation check within launch files
    try:
        rospy.init_node("steering_data_rate", anonymous=False)
        steering_data_rate_check = SteeringAngleRateChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
