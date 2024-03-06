cd /home/ros/ws_moveit/
source devel/setup.bash
export GAZEBO_MODEL_PATH=/home/ros/ws_moveit/src/regolobot/models
roslaunch moveit_robot_arm_sim full_robot_arm_sim.launch &
sleep 10
rosrun regolobot spawn_model_server.py
trap 'kill $(jobs -p)' EXIT