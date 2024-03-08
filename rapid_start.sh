cd /home/$USERNAME/ws_moveit/
source devel/setup.bash
export GAZEBO_MODEL_PATH=`pwd`/src/regolobot/models
roslaunch regolobot_moveit_config simulation.launch &
sleep 10
rosrun regolobot spawn_model_server.py
pkill ros
sleep 10
pkill gzclient