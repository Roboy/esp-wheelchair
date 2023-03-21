# this script run all the necessary setup to run the assisted navigation program using the teleop keyboard

export ROS_HOSTNAME=192.168.0.114
# if you use other ip for ros master then do  “export ROS_MASTER_URI=http://{ROS_MASTER_IP}:11311”
export ROS_MASTER_URI=http://192.168.1.105:11311
python3 ../src/assistedNavigation_aio.py 
python3 ../src/twist_to_pwm.py 
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

rosrun teleop_twist_keyboard teleop_twist_keyboard.py