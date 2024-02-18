# A script to log lateral acceleration in a txt file

import rospy
from std_msgs.msg import Float64
import numpy as np
import csv
import time

# column that tells you if you're acceleration above some value
file = open('src/buggy/scripts/validation/lateral_acc.txt', 'w', newline='')
writer = csv.writer(file)
writer.writerow(["time", "lateral accel front","lateral accel rear","average","flag front","flag rear","flag avg"])

class Later_acc:
  
    def __init__(self, velocity, steering_angle, acc):
        self.velocity = velocity
        self.steering = steering_angle
        self.acc = 0
        if self.steering!=0:
            self.radius_front_wheel = 1.104 / np.sin(np.deg2rad(self.steering))
            self.radius_rear_wheel = 1.104 / np.tan(np.deg2rad(self.steering))

        self.thres_front = 0.4
        self.thres_rear = 0.5
        self.thres_avg = 0.5

    def update_velocity(self,new_vel):
        self.velocity = new_vel

    def update_steering(self,new_steer):
        self.steering = new_steer
        if self.steering!=0:
            self.radius_front_wheel = 1.104 / np.sin(np.deg2rad(self.steering))
            self.radius_rear_wheel = 1.104 / np.tan(np.deg2rad(self.steering))

    def calculate_lateral_acc(self):
        if self.steering!=0:
            lat_accel_front_wheel = (self.velocity**2)/self.radius_front_wheel / 9.81
            lat_accel_rear_wheel = (self.velocity**2)/self.radius_rear_wheel / 9.81
            lat_accel = (lat_accel_front_wheel + lat_accel_rear_wheel) / 2

            time_rn = time.time() - start_time
            
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

            print("lateral acc is ", lat_accel)
            print("time is ",time_rn)

def callback_vel(vel_data):
    lateral_acc.update_velocity(vel_data.data)
    lateral_acc.calculate_lateral_acc()

def callback_steer(steer):
    lateral_acc.update_steering(steer.data)
    lateral_acc.calculate_lateral_acc()

def vel_subscriber():
    rospy.Subscriber("/SC/velocity", Float64, callback_vel)

def steering_subscriber():
    rospy.Subscriber("/SC/input/steering",Float64,callback_steer)
    
lateral_acc = Later_acc(0,0,0)
start_time = time.time()

if __name__ == '__main__':
    try:
        rospy.init_node("later_acc_vel")
        vel_subscriber()
        steering_subscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass