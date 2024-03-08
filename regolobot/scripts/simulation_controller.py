#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from regolobot.srv import SpawnModel
from gazebo_ros.gazebo_interface import DeleteModel
import math
from math_stick import MathStickManager

# I can only assume, given that I could not find this information anywhere, that a "unit" in Gazebo is 1 meter by convention

# TODO export GAZEBO_MODEL_PATH = package/models dir
# https://answers.ros.org/question/404423/whats-the-correct-way-to-load-mesh-files-in-gazebo-and-rviz/
# FOR SCALING https://answers.gazebosim.org/question/16774/resize-simple-objects-to-specific-sizes/

BASE_FILE_PATH = "/home/ros/ws_moveit/src/regolobot/models/mathstick"

# RANDOM COORDINATES BOUNDARIES
boundaries = {
    "min_x": 0.6,
    "max_x": 1.5,
    "min_y": -1.6,
    "max_y": -0.4,
    "min_z": 0.7,
    "max_z": 0.8,
    "min_roll": math.pi,
    "max_roll": math.pi,
    "min_pitch": math.pi / 2,
    "max_pitch": math.pi / 2,
    "min_yaw": 0,
    "max_yaw": 0,
}


def loop():
    rospy.loginfo("Waiting for Spawn Model Service to be ready")
    rospy.wait_for_service("regolobot/spawn_model")
    rospy.init_node("random_spawner")
    try:
        spawn_model_proxy = rospy.ServiceProxy("regolobot/spawn_model", SpawnModel)
        delete_model_proxy = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    except rospy.ServiceException as e:
        rospy.loginfo(f"Error while preparing service. Details {e}")
        return

    stick_manager = MathStickManager(
        spawn_model_proxy, delete_model_proxy, 10, BASE_FILE_PATH, **boundaries
    )
    while not rospy.is_shutdown():
        number = int(input("How many math sticks do you want to spawn? "))
        stick_manager.delete_all()
        for _ in range(number):
            stick_manager.spawn(stick_manager.random())
    # rate = rospy.Rate(0.5) # 0.5hz
    # while not rospy.is_shutdown():
    #    counter = spawn_random_model(spawn_model, counter)
    #    rate.sleep()


if __name__ == "__main__":
    try:
        loop()
    except rospy.ROSInterruptException:
        pass
