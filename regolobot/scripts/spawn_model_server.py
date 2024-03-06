#!/usr/bin/env python
from regolobot.srv import SpawnModel, SpawnModelResponse, SpawnModelRequest
import rospy
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from gazebo_ros.gazebo_interface import spawn_sdf_model_client
from pathlib import Path

def get_pose(r: SpawnModelRequest):
    initial_pose = Pose()
    initial_pose.position.x = r.x_pos
    initial_pose.position.y = r.y_pos
    initial_pose.position.z = r.z_pos
    q = quaternion_from_euler(r.roll, r.pitch, r.yaw)
    initial_pose.orientation = Quaternion(*q)
    return initial_pose

def spawn_model_handler(r: SpawnModelRequest):
    model_file = Path(r.model_path)
    print(r.model_path)
    print(Path.cwd())
    if model_file.is_file():
        f = open(model_file, 'r')
        model_xml = f.read()
    else:
        model_xml = ""
    rospy.loginfo(model_xml)
    initial_pose = get_pose(r)
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    try:
        params = {
            'model_name': r.name,
            'model_xml': model_xml,
            'robot_namespace': rospy.get_namespace(),
            'initial_pose': initial_pose,
            'reference_frame': '',
            'gazebo_namespace': '/gazebo'
            
        }
        success = spawn_sdf_model_client(**params)
        return SpawnModelResponse(success)
    except Exception as e:
        rospy.loginfo(f"Gazebo had an error while spawning this model. Details: {e}")
        return SpawnModelResponse(False)

def spawn_model_server():
    rospy.init_node("spawn_model_server")
    s = rospy.Service("regolobot/spawn_model", SpawnModel, spawn_model_handler)
    print("Ready to spawn models")
    rospy.spin()

if __name__ == "__main__":
    spawn_model_server()
