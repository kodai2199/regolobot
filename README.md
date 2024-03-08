<<<<<<< HEAD
COME FARE PARTIRE IL TUTTO (credo):
=======
COME FARE PARTIRE IL TUTTO (credo):  

1. un catkin build non ha mai ammazzato nessuno  
2. in tutti i terminali che apri sempre source/devel/setup.bash  
3. per aprire rviz + gazebo:   
   export GAZEBO_MODEL_PATH=/home/elisa/ws_moveit/src/regolobot/models  
   roslaunch regolobot_moveit_config simulation.launch  
4. far partire la simulazione dei regoli:  
   rosrun regolobot spawn_model_server.py (in un terminale)  
   rosrun regolobot simulation_controller.py (in un altro terminale)  
>>>>>>> af4fc5c3c70791cbac89192021dfaf5e64a897f5

un catkin build non ha mai ammazzato nessuno
in tutti i terminali che apri sempre source/devel/setup.bash
per aprire rviz + gazebo:
export GAZEBO_MODEL_PATH=/home/elisa/ws_moveit/src/regolobot/models
roslaunch regolobot_moveit_config simulation.launch
far partire la simulazione dei regoli:
rosrun regolobot spawn_model_server.py (in un terminale)
rosrun regolobot simulation_controller.py (in un altro terminale)
Altre cose utili:

dove trovare le pose predefinite per cambiarle/aggiungerle: moveit_robot_arm_sim / config / robot_arm_urdf.srdf
Group_Names: arm_group, hand
pose importanti: hand_open, hand_closed
see the parsing of robot:
cd ~/ws_moveit/src/robot_arm_urdf/urdf
check_urdf robot_arm_urdf.urdf
