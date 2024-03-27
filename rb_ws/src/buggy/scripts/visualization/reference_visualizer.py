#! /usr/bin/env python3
import argparse
import json

import rospy
from foxglove_msg.msg import GeoJSON

class geojsonVisualizer():
    def __init__(self, waypoint_file, buggy_name):
        # Read in bag path from command line
        self.ref_publisher = rospy.Publisher(buggy_name + "/reference_path", GeoJSON, queue_size=1)

        # Read the JSON file
        self.waypoint_file = waypoint_file
    
    def generate_geojson(self, coords):
        feature_collection = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {},
                    "geometry": {
                        "coordinates": coords,
                        "type": "LineString"
                    }
                }
            ]
        }
        return feature_collection
    
    def publish_msg(self):
        with open(self.waypoint_file) as f:
            data = json.load(f)

        # Print latitude and longitude coordinates
        coords = [[entry['lon'], entry['lat']] for entry in data]
        feature_collection = self.generate_geojson(coords)
        self.ref_publisher.publish(GeoJSON(json.dumps(feature_collection, ensure_ascii=False).encode('utf-8')))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--waypoint_file", type=str, help="Path to waypoint file", required=True)
    parser.add_argument("--buggy_name", type=str, help="name of ego-buggy", required=True)
    args = parser.parse_args()

    rospy.init_node("geojsonVisualizer")
    print("LAUNCHED")
    visualizer = geojsonVisualizer("/rb_ws/src/buggy/paths/" + args.waypoint_file, args.buggy_name)
    visualizer.publish_msg()
