#! /usr/bin/python3

import math
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import actionlib
from math import pi, tau, dist, fabs, cos
from colorama import Fore, Back, Style
import sys


class Regolobot:
    def __init__(self, group_name):

        # Initialize the moveit_commander and planning scene interface
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_group = group_name
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        # self._group.set_planning_time(3.0)

        # Create a DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )

        # Create action client for the "Execute Trajectory" action server
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            "execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction
        )
        self._exectute_trajectory_client.wait_for_server()

        # Get the planning frame, end effector link and the robot group names
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo(
            f"{Fore.LIGHTMAGENTA_EX}Planning Group: {self._planning_frame}{Style.RESET_ALL}"
        )
        rospy.loginfo(
            f"{Fore.LIGHTMAGENTA_EX}End Effector Link: {self._eef_link}{Style.RESET_ALL}"
        )
        rospy.loginfo(
            f"{Fore.LIGHTMAGENTA_EX}Group Names: {self._group_names}{Style.RESET_ALL}"
        )
        rospy.loginfo(
            f"{Fore.LIGHTMAGENTA_EX}MyRobot initialization is done.{Style.RESET_ALL}"
        )

    def set_pose(self, arg_pose_name, time=3):
        rospy.loginfo(f"{Fore.GREEN}Going to Pose: {arg_pose_name}{Style.RESET_ALL}")

        # for moveit_commander member functions in Python 3 (For Noetic), please refer: https://docs.ros.org/en/noetic/api/moveit_commander/html/functions_func.html
        # Python file with function definitions: https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
        # Python file with function definitions (for Noetic): https://docs.ros.org/en/noetic/api/moveit_commander/html/move__group_8py_source.html
        self._group.set_planning_time(time)
        self._group.set_named_target(arg_pose_name)

        plan_success, plan, planning_time, error_code = self._group.plan()

        # Create a goal message object for the action server
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # Update the trajectory in the goal message
        goal.trajectory = plan
        # Send the goal to the action server
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()

    def pose_error(self, pose1: Pose, pose2: Pose):
        pos_errors = [
            abs(pose1.position.x - pose2.position.x),
            abs(pose1.position.y - pose2.position.y),
            abs(pose1.position.z - pose2.position.z),
        ]
        q1 = [
            pose1.orientation.x,
            pose1.orientation.y,
            pose1.orientation.z,
            pose1.orientation.w,
        ]
        q2 = [
            pose2.orientation.x,
            pose2.orientation.y,
            pose2.orientation.z,
            pose2.orientation.w,
        ]
        pose1_rpy = euler_from_quaternion(q1)
        pose2_rpy = euler_from_quaternion(q2)
        orientation_errors = [
            abs(pose1_rpy[i] - pose2_rpy[i]) % math.pi for i in range(len(pose1_rpy))
        ]
        return max(pos_errors), max(orientation_errors)

    def is_stationary(self, tolerance_pos=0.01, tolerance_ori=0.01):
        pose_1 = self._group.get_current_pose().pose
        rospy.sleep(0.01)
        pose_2 = self._group.get_current_pose().pose
        pos_error, ori_error = self.pose_error(pose_1, pose_2)
        # print(f"From is stationary: {pos_error}, {ori_error}")
        return pos_error < tolerance_pos and ori_error < tolerance_ori

    def safe_move_ee(
        self, x, y, z, roll=0, pitch=0, yaw=0, tolerance_pos=0.02, tolerance_ori=100
    ):
        # Works for RPY only since the default is to use xyz axes instead of zyz
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_goal = Pose()
        pose_goal.orientation = Quaternion(*q)
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        rospy.loginfo(f"{Fore.GREEN}Going to Pose: ({x}, {y}, {z}){Style.RESET_ALL}")
        self._group.stop()
        self._group.clear_pose_targets()
        current_pose = self._group.get_current_pose().pose
        pos_error, ori_error = self.pose_error(current_pose, pose_goal)
        start_timeout = 2
        while pos_error > tolerance_pos or ori_error > tolerance_ori:
            wait_counter = 0
            # print(pos_error, ori_error)
            self.move_ee(pose_goal, start_timeout)
            current_pose = self._group.get_current_pose().pose
            while not self.is_stationary() and wait_counter < 2:
                rospy.sleep(0.05)
                wait_counter += 0.05
            if wait_counter >= 2:
                self._group.stop()
            pos_error, ori_error = self.pose_error(current_pose, pose_goal)
            start_timeout *= 2

    def move_ee(self, pose_goal, time=3):
        self._group.set_planning_time(time)
        self._group.set_planner_id("PRMstar")
        self._group.set_start_state_to_current_state()
        self._group.set_pose_target(pose_goal)

        plan_success, plan, planning_time, error_code = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        # Update the trajectory in the goal message
        goal.trajectory = plan
        # Send the goal to the action server
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()

    def __del__(self):
        moveit_commander.roscpp_shutdown()


def main():
    arm = Regolobot("manipulator")
    hand = Regolobot("gripper")
    arm.move_ee(0, 0.5, 1.07)
    del arm
    del hand


if __name__ == "__main__":
    main()
