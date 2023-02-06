# if you use other ip for ros master then do  “export ROS_HOSTNAME={ROS_MASTER_IP}”
export ROS_HOSTNAME=192.168.0.124
# if you use other ip for ros master then do  “export ROS_MASTER_URI=http://{ROS_MASTER_IP}:11311”
export ROS_MASTER_URI=http://192.168.0.124:11311
roscore &
python3 software/twist_to_pwm.py &
python3 software/tof.py & 
rosrun teleop_twist_keyboard teleop_twist_keyboard.py