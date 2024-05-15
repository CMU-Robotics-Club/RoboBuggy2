#! /usr/bin/env python3
import argparse
import csv

import rosbag
from tf.transformations import euler_from_quaternion


def main():
    """
    bag_file is an input that reads the path to the bag file
    output_file is an argument that reads the path to the output file
    subsample is the number of points to be selected to be converted to waypoints
    """
    # Read in bag path from command line
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file", help="Path to bag file")
    parser.add_argument("output_file", help="Path to output file")
    parser.add_argument(
        "subsample", help="Subsample rate (1 = don't skip any waypoints)", type=int
    )
    args = parser.parse_args()

    # Open bag
    bag = rosbag.Bag(args.bag_file)

    # Create data structure
    waypoints = []

    # Track index for skipping waypoints
    i = 0

    # Loop through bag
    for _, msg, _ in bag.read_messages(topics="/nav/odom"):
        # Skip waypoints
        if i % args.subsample != 0:
            i += 1
            continue
        i += 1

        # TODO: Check Orientation
        lat = msg.pose.pose.position.x
        lon = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation

        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        waypoints.append([str(lat), str(lon), str(yaw)])

    # Write to csv file
    with open(args.output_file, "w", newline="") as csvfile:
        writer = csv.writer(
            csvfile, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL
        )
        for row in waypoints:
            writer.writerow(row)


if __name__ == "__main__":
    main()
