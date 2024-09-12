# this script run all the necessary setup to run the assisted navigation program using the teleop keyboard

export ROS_HOSTNAME=192.168.0.114
# if you use other ip for ros master then do  “export ROS_MASTER_URI=http://{ROS_MASTER_IP}:11311”
export ROS_MASTER_URI=http://192.168.1.105:11311
python3 ../src/drive_controller.py 
python3 ../src/tof_handler.py
python3 ../src/twist_to_pwm.py 

#run gazebo world of your choice
roslaunch gazebo_ros shapes_world.launch

# on a new terminal to spawn the robody entity
roslaunch robody_sim spawn.launch
roslaunch robody_sim spawndefault.launch

rosrun gazebo spawn_model -file /home/ronggurmwp/robody_nav_ws/src/robody_sim/urdf/human.urdf -urdf -z 1 -model my_object

rosrun gazebo spawn_model -file "/home/ronggurmwp/robody_nav_ws/src/robody_sim/urdf/human.urdf" -urdf -z 1 -model my_object

rosrun teleop_twist_keyboard teleop_twist_keyboard.py