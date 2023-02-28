#! /usr/bin/env python3
import rosbag
from geometry_msgs.msg import Point
import pandas as pd
import numpy as np
import pymap3d as pm
from tf.transformations import euler_from_quaternion

bag = rosbag.Bag('/rb_ws/src/buggy/bags/trim.bag')
topic = '/nav/odom'

LATITUDE_OFFSET = 40.441687
LONGITUDE_OFFSET = -79.944276

is_first_iter = True
start_time = 0
time_array = []
latitude_array = []
longitude_array = []
down_array = []
heading_array = []
x_array = []
y_array = []
z_array = [] # NOTE: Calculation for this needs to be edited

for topic, msg, t in bag.read_messages(topics=topic):
    if (is_first_iter):
        start_time = msg.header.stamp
        is_first_iter = False
    current_time = msg.header.stamp - start_time

    pose = msg.pose.pose.position
    latitude = pose.x
    longitude = pose.y

    # swap quarternion axes to convert
    # from NED orientation to ENU
    q_x = msg.pose.pose.orientation.y
    q_y = msg.pose.pose.orientation.x
    q_z = msg.pose.pose.orientation.z
    q_w = msg.pose.pose.orientation.w

    (_, _, heading_deg) = np.rad2deg(euler_from_quaternion([q_x, q_y, q_z, q_w]))

    # # 111,111 m per degree of latitude
    # # 111,111 * cos(latitude) m per degree of longitude
    # x = (latitude - LATITUDE_OFFSET) * 111111.0
    # y = (longitude - LONGITUDE_OFFSET) * np.cos(longitude) * 111111.0

    z = pose.z
    
    (e, n, u) = pm.geodetic2enu(latitude,
        longitude, 
        z, 
        LATITUDE_OFFSET, 
        LONGITUDE_OFFSET, 
        0, 
        ell=None, 
        deg=True)

    time_array.append(current_time)
    latitude_array.append(latitude)
    longitude_array.append(longitude)
    down_array.append(z)
    heading_array.append(heading_deg)
    x_array.append(e)
    y_array.append(n)
    z_array.append(u)

df = pd.DataFrame(
        {'time': time_array,
         'latitude': latitude_array,
         'longitude': longitude_array,
         'down': down_array,
         'heading': heading_array,
         'x': x_array,
         'y': y_array,
         'z': z_array})

df.to_csv('/rb_ws/src/buggy/paths/out.csv')