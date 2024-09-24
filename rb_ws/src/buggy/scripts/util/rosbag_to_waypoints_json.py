#! /usr/bin/env python3
import argparse
import uuid
import json

import rosbag

#BROKEN BROKEN BROKEN

def main():
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

        lon = msg.pose.pose.position.x
        lat = msg.pose.pose.position.y



        waypoints.append(
            {
                "key": str(uuid.uuid4()),
                "lat": lat,
                "lon": lon,
                "active": False,
            }
        )

    # Write to JSON file
    with open(args.output_file, "w") as f:
        json.dump(waypoints, f, indent=4)


if __name__ == "__main__":
    main()
