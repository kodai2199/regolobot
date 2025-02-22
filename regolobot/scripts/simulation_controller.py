#!/usr/bin/env python

import sys
import rospy
from regolobot.srv import SpawnModel
from gazebo_ros.gazebo_interface import DeleteModel
import math
from pathlib import Path
from math_stick import MathStickManager
from image_processing import ImageConverter
from robot_interface import Regolobot
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from prediction_client import predict_image
from solution import Solution, backtrack_solution

BASE_MODEL_PATH = str(Path.cwd() / "src/regolobot/models/mathstick")

# RANDOM COORDINATES BOUNDARIES
boundaries = {
    "min_x": -0.8,
    "max_x": -0.05,
    "min_y": 0.2,
    "max_y": 0.9,
    "min_z": 0.91,
    "max_z": 0.92,
    "min_roll": 0,
    "max_roll": 0,
    "min_pitch": 0,
    "max_pitch": 0,
    "min_yaw": 0,
    "max_yaw": math.pi,
}

# SIMULATION STEPS
# ? Task mode (target, order)
# ? Input for resetting?
# 1. Decide whether use random or precise mathstick spawn
# 2. In any case, spawn the requested mathsticks
# 3. Send iamges to the NN for recognition
# 4. Determine solution using available mathsticks
# 5. Ensure solution is feasible (space, available mathsticks)
# 6. Start from the base (x,y) coordinates and start building the solution
# 7. At the end, check if the robot successfully built the solution
# 8. Ask for reset/reorder


def current_value(labels: list):
    current = 0
    labels = sorted(labels, key=lambda e: (e["x"], e["y"]))
    for label in labels:
        print(label)
        if label["x"] < 0:
            # This stick is not in the "Place" area
            continue


def input_spawn_parameters():
    print("\033[H\033[J", end="")
    math_sticks = None
    choice = input(
        "Would you like to choose the math sticks to spawn? (yes/no). If you choose 'no' they will spawn randomly. Type 'exit' to stop: "
    ).lower()
    if choice == "y" or choice == "yes":
        sticks_input = input(
            "Insert the numbers that identify the math sticks you prefer (separated by spaces): "
        )
        math_sticks = [int(stick) for stick in sticks_input.split()]
        num_sticks = len(math_sticks)
    elif choice == "n" or choice == "no":
        num_sticks = int(
            input("Insert the total number of math sticks to randomly generate: ")
        )
    elif choice == "exit":
        sys.exit()
    else:
        print("Invalid choice. Please enter 'y' or 'n'.")
        return input_spawn_parameters()
    return num_sticks, math_sticks


def input_target():
    target = input("Insert the new target value, or type 'back': ")
    if target == "back":
        return None
    return int(target)


def addend_height(addend):
    factors = len(addend)
    return 0.01 * factors + 0.04 * (factors - 1)


def addend_width(addend, labels):
    factors = [labels[factor]["category"] for factor in addend]
    max_factor = max(factors)
    return 0.01 * max_factor


def publish_table():
    # Create CollisionObject message
    collision_object = CollisionObject()
    collision_object.header.frame_id = "world"

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
    collision_object_publisher = rospy.Publisher(
        "/collision_object", CollisionObject, queue_size=10, latch=True
    )
    collision_object_publisher.publish(collision_object)


def execute_solution(arm, gripper, solution: Solution, labels, start_x, center_y):
    arm.set_pose("home")
    gripper.set_pose("open", 1)
    rospy.sleep(1)
    # 1. Determine max solution height
    max_height = max([addend_height(addend) for addend in solution._hierarchy])
    print(f"Total height {max_height}")
    # Height is the total height of the sticks plus the space between them
    if max_height < 0:
        raise ValueError("Max height is negative. There was surely an error.")
    if max_height > 0.8:
        raise ValueError("The solution is too tall. It will not fit on the table.")

    # 2. Determine solution length
    length = 0
    for addend in solution._hierarchy:
        length += addend_width(addend, labels)
    # Space between addends
    length += 0.02 * (len(solution._hierarchy) - 1)
    print(f"Total length {length}")
    if length < 0:
        raise ValueError("Solution length is negative. There was surely an error.")
    if length > 0.9:
        raise ValueError("The solution is too long. It will not fit on the table.")

    current_x = start_x
    for addend in solution._hierarchy:
        height = addend_height(addend)
        current_y = center_y - height / 2
        for factor in addend:
            gripper.set_pose("open", 2)
            stick = labels[factor]
            roll = stick.get("roll", 0)
            pitch = math.pi / 2
            yaw = stick.get("yaw", 0)

            # 3. Move to the stick
            # arm.move_ee(stick["x"], stick["y"], 1.15, roll, pitch, yaw)
            arm.safe_move_ee(stick["x"], stick["y"], 1.066, roll, pitch, yaw)
            gripper.set_pose("closed", 1)
            rospy.sleep(1)

            if len(addend) == 1:
                # Element must be placed vertically
                # since it's just a sum
                target_yaw = math.pi / 2
            else:
                target_yaw = 0
            arm.safe_move_ee(
                (stick["x"] + current_x) / 2,
                (current_y + stick["y"]) / 2,
                1.4,
                roll,
                pitch,
                (target_yaw + yaw) / 2,
            )
            arm.safe_move_ee(current_x, current_y, 1.07, roll, pitch, target_yaw)
            gripper.set_pose("open", 2)
            rospy.sleep(1)
            arm.safe_move_ee(current_x, current_y, 1.25, roll, pitch, 0)
            # arm.move_ee(0, 0.55, 1.15, roll, pitch, 0)
            # 5. Move to the next addend
            current_y += 0.05
        current_x += addend_width(addend, labels) + 0.02
    arm.set_pose("zero_pose")


def loop():
    rospy.loginfo("Waiting for Spawn Model Service to be ready")
    rospy.wait_for_service("regolobot/spawn_model")
    rospy.init_node("simulation_controller")
    publish_table()

    try:
        spawn_model_proxy = rospy.ServiceProxy("regolobot/spawn_model", SpawnModel)
        delete_model_proxy = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    except rospy.ServiceException as e:
        rospy.loginfo(f"Error while preparing service. Details {e}")
        return

    color_converter = ImageConverter("camera/color/image_raw", ".")
    depth_converter = ImageConverter("camera/depth/image_raw", ".")

    arm = Regolobot("manipulator")
    hand = Regolobot("gripper")
    # arm.move_ee(0, 0.5, 1.07)

    stick_manager = MathStickManager(
        spawn_model_proxy, delete_model_proxy, 10, BASE_MODEL_PATH, **boundaries
    )

    """
    arm.set_pose("home")
    input("Press Enter to continue...")
    arm.move_ee(0, 0.55, 1.3, 0, math.pi / 2, math.pi / 2)
    input("Press Enter to continue...")
    arm.move_ee(0, 0.55, 1.3, 0, math.pi / 2, 0)
    input("Press Enter to continue...")
    arm.move_ee(0, 0.55, 1.3, 0, math.pi / 2, -math.pi / 2)
    print("Checking ROLL")
    input("Press Enter to continue...")
    arm.move_ee(0, 0.55, 1.3, math.pi / 2, math.pi / 2, 0)
    input("Press Enter to continue...")
    arm.move_ee(0, 0.55, 1.3, 0, math.pi / 2, 0)
    input("Press Enter to continue...")
    arm.move_ee(0, 0.55, 1.3, -math.pi / 2, math.pi / 2, 0)
    return
    """
    while not rospy.is_shutdown():
        arm.set_pose("zero_pose")
        stick_manager.delete_all()  # TODO enable only on reset
        num_sticks, math_sticks = input_spawn_parameters()
        if num_sticks == 0:
            sys.exit()

        # Spawn sticks
        if math_sticks is None:
            for _ in range(num_sticks):
                stick_manager.spawn(stick_manager.random_within(0.8))
            math_sticks = [stick.category for stick in stick_manager.spawned_models]
        else:
            # TODO Error if e not in range (1, 11)
            for e in math_sticks:
                stick_manager.spawn(stick_manager.random_within(0.8, category=e))
        rospy.sleep(1)
        math_sticks.sort(reverse=True)
        print(f"Spawned {len(stick_manager.spawned_models)} math sticks: {math_sticks}")

        # Check for correspondence with recognition
        success, labels = predict_image(color_converter.latest)
        labels.sort(key=lambda e: (e["category"]), reverse=True)
        if success:
            print(
                f"The system detected the following math sticks: {[l['category'] for l in labels]}"
            )
        else:
            print("Network or image acquisition error. Please try again.")
            continue

        # Enter target and compute solution
        target = input_target()
        if target is None:
            continue
        print(f"Target: {target}")
        math_sticks = [l["category"] for l in labels]
        solution = backtrack_solution(math_sticks, target)
        if solution is None:
            print("The selected target cannot be reached using available sticks")
            continue
        print(solution.solution)
        print(solution._hierarchy)
        execute_solution(arm, hand, solution, labels, 0.1, 0.55)
        input("Press Enter to continue...")


if __name__ == "__main__":
    print("Hello! This is RegoloBot.")
    try:
        loop()
    except rospy.ROSInterruptException:
        pass
