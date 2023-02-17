# this script run all the necessary setup to run the assisted navigation program using the teleop keyboard

export ROS_HOSTNAME=192.168.0.124
# if you use other ip for ros master then do  “export ROS_MASTER_URI=http://{ROS_MASTER_IP}:11311”
export ROS_MASTER_URI=http://192.168.0.124:11311
roscore &
python3 ../src/tof.py & 
rosrun teleop_twist_keyboard teleop_twist_keyboard.py