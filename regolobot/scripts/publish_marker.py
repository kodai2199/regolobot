#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

rospy.init_node('rviz_marker')

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

marker = Marker()

marker.header.frame_id = "map"
marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker.type = 1
marker.id = 0

# Set the scale of the marker
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 1.0

# Set the color
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0

# Set the pose of the marker
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

while not rospy.is_shutdown():
  marker_pub.publish(marker)
  rospy.rostime.wallsleep(1.0)
