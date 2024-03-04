COME FARE PARTIRE IL TUTTO (credo):  

ELISA
1. un catkin build non ha mai ammazzato nessuno  
2. in tutti i terminali che apri sempre source/devel/setup.bash  
3. per aprire rviz + gazebo:   
   export GAZEBO_MODEL_PATH=/home/elisa/ws_moveit/src/regolobot/models  
   roslaunch moveit_robot_arm_sim full_robot_arm_sim.launch  
5. far partire la simulazione dei regoli:  
   rosrun regolobot spawn_model_server.py (in un terminale)  
   rosrun regolobot simulation_controller.py (in un altro terminale)  
6. far partire la simulazione del robot:
   chmod a+x /home/$USERNAME/ws_moveit/src/moveit_robot_arm_sim/node_set_prefined_pose.py 
   rosrun moveit_robot_arm_sim node_set_prefined_pose.py (altro terminale)

GIULIO:
1. un catkin build non ha mai ammazzato nessuno  
2. in tutti i terminali che apri sempre source/devel/setup.bash  
3. per aprire rviz + gazebo:   
   export GAZEBO_MODEL_PATH=/home/giulio/ws_moveit/src/regolobot/models  
   roslaunch moveit_robot_arm_sim full_robot_arm_sim.launch  
5. far partire la simulazione dei regoli:  
   rosrun regolobot spawn_model_server.py (in un terminale)  
   rosrun regolobot simulation_controller.py (in un altro terminale)  
6. far partire la simulazione del robot:
   chmod a+x /home/$USERNAME/ws_moveit/src/moveit_robot_arm_sim/node_set_prefined_pose.py 
   rosrun moveit_robot_arm_sim node_set_prefined_pose.py (altro terminale)


Altre cose utili:
- dove trovare le pose predefinite per cambiarle/aggiungerle: moveit_robot_arm_sim / config / robot_arm_urdf.srdf
- Group_Names: arm_group, hand
- pose importanti: hand_open, hand_closed
- see the parsing of robot:  
  cd ~/ws_moveit/src/robot_arm_urdf/urdf  
  check_urdf robot_arm_urdf.urdf
