import rosbag
from geometry_msgs.msg import Point
import pandas as pd
import numpy as np
from tf.transformations import euler_from_quaternion

bag = rosbag.Bag('/my_desktop/2023-02-12-11-39-27.bag')
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
    down = pose.z

    q_x = msg.pose.pose.orientation.x
    q_y = msg.pose.pose.orientation.y
    q_z = msg.pose.pose.orientation.z
    q_w = msg.pose.pose.orientation.w
    (_, _, heading) = np.rad2deg(euler_from_quaternion([q_x, q_y, q_z, q_w]))

    # 111,111 m per degree of latitude
    # 111,111 * cos(latitude) m per degree of longitude
    x = (latitude - LATITUDE_OFFSET) * 111111.0
    y = (longitude - LONGITUDE_OFFSET) * np.cos(longitude) * 111111.0
    z = down # NOTE: NEED TO CHANGE <-------------------------------------------


    time_array.append(current_time)
    latitude_array.append(latitude)
    longitude_array.append(longitude)
    down_array.append(down)
    heading_array.append(heading)
    x_array.append(x)
    y_array.append(y)
    z_array.append(z)

df = pd.DataFrame(
        {'time': time_array,
         'latitude': latitude_array,
         'longitude': longitude_array,
         'down': down_array,
         'heading': heading_array,
         'x': x_array,
         'y': y_array,
         'z': z_array})

df.to_csv('out.csv')