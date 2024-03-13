cd /home/$USERNAME/ws_moveit/
source devel/setup.bash
export GAZEBO_MODEL_PATH=`pwd`/src/regolobot/models
roslaunch regolobot_moveit_config simulation.launch >/dev/null 2>&1 &
sleep 10
rosrun regolobot spawn_model_server.py >/dev/null 2>&1 &
rosrun regolobot simulation_controller.py
pkill ros
pkill roscore
pkill gzserver
pkill gzclient
