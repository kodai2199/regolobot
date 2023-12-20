#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from regolobot.srv import SpawnModel
import random

#TODO export GAZEBO_MODEL_PATH = package/models dir
#https://answers.ros.org/question/404423/whats-the-correct-way-to-load-mesh-files-in-gazebo-and-rviz/
# FOR SCALING https://answers.gazebosim.org/question/16774/resize-simple-objects-to-specific-sizes/

BASE_FILE_PATH = "models/regolo"
regoli = [i for i in range(1, 11)]


def spawn_random_model(proxy, counter):
    regolo = random.choice(regoli)
    nome = f"regolo{counter}"
    file = f"{BASE_FILE_PATH}{regolo}/model.sdf"
    x = random.random() * 3
    y = random.random() * 3
    z = random.random() * 3
    roll = random.random() * 5
    pitch = random.random() * 5
    yaw = random.random() * 5
    try:
        proxy(nome, file, x, y, z, roll, pitch, yaw, 0, 0, 0)
        rospy.loginfo(f"Spawned new regolo{regolo}")
        counter += 1
        return counter
    except rospy.ServiceException as e:
        rospy.loginfo(f"Error while calling Spawn Model Service. Details: {e}")
        return counter
    


def loop():
    counter = 0
    rospy.loginfo("Waiting for Spawn Model Service to be ready")
    rospy.wait_for_service('regolobot/spawn_model')
    rospy.init_node('random_spawner')
    try:
        spawn_model = rospy.ServiceProxy('regolobot/spawn_model', SpawnModel)
    except rospy.ServiceException as e:
        rospy.loginfo(f"Error while preparing service. Details {e}")
        return
    rate = rospy.Rate(0.5) # 0.5hz
    while not rospy.is_shutdown():
        counter = spawn_random_model(spawn_model, counter)
        rate.sleep()

if __name__ == '__main__':
    try:
        loop()
    except rospy.ROSInterruptException:
        pass