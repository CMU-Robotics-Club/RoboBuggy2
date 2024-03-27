#! /usr/bin/env python3
import rosbag
import argparse
import uuid
import json
import csv
from tf.transformations import euler_from_quaternion
import os

def main():
    # Read in bag path from command line
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", help="Directory of bag files")
    parser.add_argument("output_directory", help="Path to output directory")
    parser.add_argument(
        "subsample", help="Subsample rate (1 = don't skip any waypoints)", type=int
    )
    args = parser.parse_args()

    directory = args.directory
    files = os.listdir(directory)
    for file_name in files:
        file_path = os.path.join(directory, file_name)
        if os.path.isfile(file_path):
            bag = rosbag.Bag(file_path)
            # Create data structure
            waypoints = []

            # Track index for skipping waypoints
            i = 0

            # Loop through bag
            for topic, msg, t in bag.read_messages(topics="/nav/odom"):
                # Skip waypoints
                if i % args.subsample != 0:
                    i += 1
                    continue
                i += 1

                o
                lat = msg.pose.pose.position.x
                lon = msg.pose.pose.position.y
                orientation_q = msg.pose.pose.orientation

                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion (orientation_list)


                waypoints.append([str(lat), str(lon), str(yaw)])
            print(file_path)
    
    return


    # Open bag
    bag = rosbag.Bag(args.bag_file)

    # Create data structure
    waypoints = []

    # Track index for skipping waypoints
    i = 0

    # Loop through bag
    for topic, msg, t in bag.read_messages(topics="/nav/odom"):
        # Skip waypoints
        if i % args.subsample != 0:
            i += 1
            continue
        i += 1

        lat = msg.pose.pose.position.x
        lon = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation

        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)


        waypoints.append([str(lat), str(lon), str(yaw)])

    # Write to csv file
    with open(args.output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for row in waypoints:
            writer.writerow(row)
        


if __name__ == "__main__":
    main()
