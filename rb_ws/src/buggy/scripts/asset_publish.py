#!/usr/bin/env python3
import rospy

from visualization_msgs.msg import Marker

rospy.init_node("asset_publish")

mesh_pub = rospy.Publisher("foxglove/mesh", Marker, queue_size=10)

# Publish Mesh infrequently
marker = Marker()

marker.header.frame_id = "base"
marker.header.stamp = rospy.Time.now()
marker.ns = ""

# Shape (resource type = 10 (mesh))
marker.type = 10
marker.id = 0
marker.action = 0

# Note: Must set mesh_resource to a valid URL for a model to appear
marker.mesh_resource = "http://127.0.0.1:8760/cmutopo.dae"
marker.mesh_use_embedded_materials = True

# Scale
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 1.0

# Color
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 0.0
marker.color.a = 1.0

# Pose
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 0.0

RATE = 2
rate = rospy.Rate(RATE)

while not rospy.is_shutdown():
  mesh_pub.publish(marker)
  rate.sleep()


  

