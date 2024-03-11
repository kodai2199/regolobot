#!/usr/bin/env python
import rospy
from regolobot.srv import SpawnModel
from gazebo_ros.gazebo_interface import DeleteModel
import math
from math_stick import MathStickManager
from image_processing import ImageConverter
from pathlib import Path

BASE_MODEL_PATH = str(Path.cwd() / "src/regolobot/models/mathstick")

# RANDOM COORDINATES BOUNDARIES
boundaries = {
    "min_x": -0.8,
    "max_x": 0.8,
    "min_y": 0.2,
    "max_y": 0.9,
    "min_z": 0.911,
    "max_z": 0.915,
    "min_roll": 0,
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
        spawn_model_proxy, delete_model_proxy, 10, BASE_MODEL_PATH, **boundaries
    )

    while not rospy.is_shutdown():
        number = int(input("How many mathsticks do you want to spawn? "))
        stick_manager.delete_all()
        for _ in range(number):
            stick_manager.spawn(stick_manager.random())
        print(f"Spawned {len(stick_manager.spawned_models)} mathsticks")


if __name__ == "__main__":
    try:
        loop()
    except rospy.ROSInterruptException:
        pass
