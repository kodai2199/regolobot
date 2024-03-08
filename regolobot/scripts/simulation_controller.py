#!/usr/bin/env python
import rospy
from regolobot.srv import SpawnModel
from gazebo_ros.gazebo_interface import DeleteModel
import math
from math_stick import MathStickManager
from image_processing import ImageConverter
import random
from pathlib import Path

# I can only assume, given that I could not find this information anywhere, that a "unit" in Gazebo is 1 meter by convention

# https://answers.ros.org/question/404423/whats-the-correct-way-to-load-mesh-files-in-gazebo-and-rviz/
# FOR SCALING https://answers.gazebosim.org/question/16774/resize-simple-objects-to-specific-sizes/


BASE_MODEL_PATH = str(Path.cwd() / "src/regolobot/models/mathstick")
BASE_OUTPUT_DIR = Path.cwd() / "src/regolobot/train"
# RANDOM COORDINATES BOUNDARIES
boundaries = {
    "min_x": -0.7,
    "max_x": 0.7,
    "min_y": 0.3,
    "max_y": 0.9,
    "min_z": 0.92,
    "max_z": 1,
    "min_roll": 0,
    "max_roll": math.pi,
    "min_pitch": math.pi / 2,
    "max_pitch": math.pi / 2,
    "min_yaw": 0,
    "max_yaw": 0,
}


def loop():
    color_path = BASE_OUTPUT_DIR / "color"
    depth_path = BASE_OUTPUT_DIR / "depth"
    label_path = BASE_OUTPUT_DIR / "label"
    color_path.mkdir(parents=True, exist_ok=True)
    depth_path.mkdir(parents=True, exist_ok=True)
    label_path.mkdir(parents=True, exist_ok=True)
    rospy.loginfo("Waiting for Spawn Model Service to be ready")
    rospy.wait_for_service("regolobot/spawn_model")
    rospy.init_node("random_spawner")
    try:
        spawn_model_proxy = rospy.ServiceProxy("regolobot/spawn_model", SpawnModel)
        delete_model_proxy = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    except rospy.ServiceException as e:
        rospy.loginfo(f"Error while preparing service. Details {e}")
        return
    color_converter = ImageConverter("/camera/color/image_raw", color_path)
    depth_converter = ImageConverter("/camera/depth/image_raw", depth_path, depth=True)
    stick_manager = MathStickManager(
        spawn_model_proxy, delete_model_proxy, 10, BASE_MODEL_PATH, **boundaries
    )

    epochs = int(input("How many training images do you want to create? "))
    min_mathsticks = int(
        input(
            "What is the MINIMUM amount of mathsticks would you like to spawn in each image? "
        )
    )
    max_mathsticks = int(
        input(
            "What is the MAXIMUM amount of mathsticks would you like to spawn in each image? "
        )
    )
    while not rospy.is_shutdown():
        for epoch in range(0, epochs):
            number = random.randrange(min_mathsticks, max_mathsticks + 1)
            stick_manager.delete_all()
            for _ in range(number):
                stick_manager.spawn(stick_manager.random())
            print(
                f"Spawned {len(stick_manager.spawned_models)} mathsticks for image {epoch + 1}"
            )
            stick_manager.save_labels(label_path / f"{epoch}.txt")
            color_converter.save()
            depth_converter.save()
        stick_manager.delete_all()
        break


if __name__ == "__main__":
    try:
        loop()
    except rospy.ROSInterruptException:
        pass
