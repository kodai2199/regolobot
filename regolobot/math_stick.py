from pathlib import Path
import random
from regolobot.srv import SpawnModel
from gazebo_ros.gazebo_interface import DeleteModel
import math
import rospy
import json


class MathStick:
    def __init__(
        self,
        category,
        number,
        x,
        y,
        z,
        roll=math.pi,
        pitch=math.pi / 2,
        yaw=0,
        base_model_path="",
    ):
        self.category = category
        self.number = number
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.spawned = False
        self.base_model_path = base_model_path

    @property
    def name(self):
        return f"math_stick{self.number}"

    @property
    def file(self):
        return f"{self.base_model_path}{self.category}/model.sdf"

    def label_dict(self):
        label_dict = {
            "category": self.category,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
        }
        return label_dict


class MathStickManager:
    def __init__(
        self,
        spawner_proxy: SpawnModel,
        delete_proxy: DeleteModel,
        categories_count: int,
        base_model_path: str,
        **kwargs,
    ):
        self.spawner_proxy = spawner_proxy
        self.delete_proxy = delete_proxy
        self.categories = [i for i in range(1, categories_count + 1)]
        self.spawned_models = []
        self.spawned_counter = 0
        self.base_model_path = base_model_path
        self.min_x = kwargs.get("min_x", -10)
        self.max_x = kwargs.get("max_x", 10)
        self.min_y = kwargs.get("min_y", -10)
        self.max_y = kwargs.get("max_y", 10)
        self.min_z = kwargs.get("min_z", 0)
        self.max_z = kwargs.get("max_z", 10)
        self.min_roll = kwargs.get("min_roll", 0)
        self.max_roll = kwargs.get("max_roll", 2 * math.pi)
        self.min_pitch = kwargs.get("min_pitch", 0)
        self.max_pitch = kwargs.get("max_pitch", math.pi / 2)
        self.min_yaw = kwargs.get("min_yaw", 0)
        self.max_yaw = kwargs.get("max_yaw", 0)

    def generate_coordinates(self):
        x = random.uniform(self.min_x, self.max_x)
        y = random.uniform(self.min_y, self.max_y)
        z = random.uniform(self.min_z, self.max_z)
        return x, y, z

    def generate_angles(self):
        roll = random.uniform(self.min_roll, self.max_roll)
        pitch = random.uniform(self.min_pitch, self.max_pitch)
        yaw = random.uniform(self.min_yaw, self.max_yaw)
        return roll, pitch, yaw

    def random(self):
        category = random.choice(self.categories)
        number = self.spawned_counter
        x, y, z = self.generate_coordinates()
        roll, pitch, yaw = self.generate_angles()
        return MathStick(
            category, number, x, y, z, roll, pitch, yaw, self.base_model_path
        )

    def spawn(self, math_stick: MathStick):
        name = math_stick.name
        file = math_stick.file
        x = math_stick.x
        y = math_stick.y
        z = math_stick.z
        roll = math_stick.roll
        pitch = math_stick.pitch
        yaw = math_stick.yaw
        try:
            self.spawner_proxy(name, file, x, y, z, roll, pitch, yaw, 0, 0, 0)
            rospy.loginfo(f"Spawned a new math stick: {name}")
            self.spawned_models.append(math_stick)
            self.spawned_counter += 1
            rospy.sleep(0.5)
        except rospy.ServiceException as e:
            rospy.loginfo(f"Error while calling Spawn Model Service. Details: {e}")
        return self.spawned_counter

    def current_labels(self):
        labels = []
        for stick in self.spawned_models:
            labels.append(stick.label_dict())
        return labels

    def save_labels(self, output_path: Path):
        with open(output_path, "w") as f:
            json.dump(self.current_labels(), f)

    def delete(self, name: str):
        try:
            self.delete_proxy(name)
            rospy.loginfo(f"Deleted model {name}")
            self.spawned_models = [
                model for model in self.spawned_models if model.name != name
            ]
        except rospy.ServiceException as e:
            rospy.loginfo(f"Error while calling Delete Model Service. Details: {e}")

    def delete_all(self):
        for model in [m for m in self.spawned_models]:
            self.delete(model.name)
