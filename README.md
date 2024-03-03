COME FARE PARTIRE IL TUTTO (credo):  

oss: le due cartelle che ho aggiunto moveit_robot_arm_sim e robot_arm_urdf forse devono essere due pacchetti separati, quindi forse vanno messi in <workspace moveit>/src  

1. un catkin build non ha mai ammazzato nessuno  
2. in tutti i terminali che apri sempre source/devel/setup.bash  
3. per aprire rviz + gazebo:   
   export GAZEBO_MODEL_PATH=/home/*nome*/ws_moveit/src/regolobot/models  
   roslaunch moveit_robot_arm_sim full_robot_arm_sim.launch  
5. far partire la simulazione dei regoli:  
   rosrun regolobot spawn_model_server.py (in un terminale)  
   rosrun regolobot simulation_controller.py (in un altro terminale)  
6. far partire la simulazione del robot:  
   rosrun moveit_robot_arm_sim node_set_prefined_pose.py (altro terminale)
