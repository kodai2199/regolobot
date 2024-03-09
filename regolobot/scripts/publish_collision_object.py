#!/usr/bin/env python

import rospy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

rospy.init_node('add_collision_object_example')

# Create CollisionObject message
collision_object = CollisionObject()
collision_object.header.frame_id = 'base_link'  

collision_object.id = "table"

# Define a box to add to the world
primitive = SolidPrimitive()
primitive.type = primitive.BOX
primitive.dimensions = [1.8, 0.9, 0.9]

# A pose for the box (specified relative to frame_id)
box_pose = Pose()
box_pose.orientation.w = 1.0
box_pose.position.x = 0
box_pose.position.y = 0.55
box_pose.position.z = 0.45

collision_object.primitives.append(primitive)
collision_object.primitive_poses.append(box_pose)
collision_object.operation = CollisionObject.ADD

# Create a list of CollisionObject
collision_objects = [collision_object]

# Publish CollisionObject to MoveIt
collision_object_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10, latch=True)
collision_object_publisher.publish(collision_object)

rospy.sleep(2) 

rospy.loginfo("Collision object added")


