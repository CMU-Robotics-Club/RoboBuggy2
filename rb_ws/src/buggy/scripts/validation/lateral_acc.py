# A script to log lateral acceleration in a txt file

import rospy
from std_msgs.msg import Float64
import numpy as np
import csv
import time

file = open("/rb_ws/src/buggy/scripts/validation/lateral_acc.txt", 'w', newline='')
writer = csv.writer(file)
writer.writerow(["time", "lateral accel front","lateral accel rear","average","flag front","flag rear"])

class LateralAcc:
  
    def __init__(self):
        self.velocity = 0
        self.steering = 0
        self.wheelbase = 1.104

        if self.steering != 0:
            self.radius_front_wheel = self.wheelbase / np.sin(np.deg2rad(self.steering))
            self.radius_rear_wheel = self.wheelbase / np.tan(np.deg2rad(self.steering))
        # in m/s^2
        self.thresh_front = 0.9*9.81
        self.thresh_rear = 0.6*9.81
        self.thresh_avg = 0.75*9.81
        self.start_time = time.time()
        rospy.Subscriber("/SC/velocity", Float64, self.callback_vel)
        rospy.Subscriber("/SC/input/steering",Float64,self.callback_steer)

    def update_velocity(self,new_vel):
        self.velocity = new_vel

    def update_steering(self,new_steer):
        self.steering = new_steer
        if self.steering != 0:
            self.radius_front_wheel = self.wheelbase / np.sin(np.deg2rad(self.steering))
            self.radius_rear_wheel = self.wheelbase / np.tan(np.deg2rad(self.steering))

    def calculate_lateral_acc(self):
        if self.steering != 0:
            lat_accel_front_wheel = (self.velocity**2)/self.radius_front_wheel / 9.81
            lat_accel_rear_wheel = (self.velocity**2)/self.radius_rear_wheel / 9.81
            lat_accel = (lat_accel_front_wheel + lat_accel_rear_wheel) / 2

            time_rn = time.time() - self.start_time
            # flags, if values exceed thresholds write 1
            front_thresh_val = 0
            rear_thresh_val = 0
            avg_thresh_val = 0
            if lat_accel_front_wheel > np.absolute(self.thresh_front):
                front_thresh_val = 1
            if lat_accel_rear_wheel > np.absolute(self.thresh_rear):
                rear_thresh_val = 1
            if lat_accel > np.absolute(self.thresh_avg):
                avg_thresh_val = 1

            writer.writerow([time_rn, lat_accel_front_wheel,lat_accel_rear_wheel,lat_accel,front_thresh_val,rear_thresh_val,avg_thresh_val])

    def callback_vel(self, vel_data):
        lateral_acc.update_velocity(vel_data.data)
        lateral_acc.calculate_lateral_acc()

    def callback_steer(self, steer):
        lateral_acc.update_steering(steer.data)
        lateral_acc.calculate_lateral_acc()
    
if __name__ == '__main__':
    try:
        lateral_acc = LateralAcc()
        rospy.init_node("later_acc_vel")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass