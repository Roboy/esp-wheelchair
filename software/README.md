# ToF Setup

## Variables : 

MAC ADDRESS ToF 1 : 3C:FB:96:DC:59:7E
MAC ADDRESS ToF 2 : 3C:FB:96:DC:59:7F

ROS_MASTER_URI=http://192.168.0.104:11311

For this documentation we use this Router Configuration : 


TOF1
MAC ADDRESS: 3C:FB:96:DC:59:7E
IP: 192.168.0.122

TOF2
MAC ADDRESS: 3C:FB:96:DC:59:7F
IP: 192.168.0.123

MASTER 
IP : 192.168.0.124

## Setup TOF 1

to setup tof1 do these command :

cd etc
nano rc.local

and then add rc.local with these commands

...

#stops royale viewer entity
systemctl stop royaleviewer
source /opt/ros/melodic/setup.sh

# if you use other ip for ros master then do  “export ROS_MASTER_URI=http://{ROS_MASTER_IP}:11311”
export ROS_MASTER_URI=http://192.168.0.124:11311

# if you use other ip for ToF2 then do  “export ROS_HOSTNAME={TOF2_IP}”
export ROS_HOSTNAME=192.168.0.122
roslaunch royale_ros_sample camera_driver.launch node_name:=”tof1”

exit 0

Reset the ToF sensor then ToF should publish data to /tof1/...

## Setup TOF 2

to setup tof2 do these command :

cd etc
nano rc.local

and then add rc.local with these commands

...

#stops royale viewer entity
systemctl stop royaleviewer
source /opt/ros/melodic/setup.sh

# if you use other ip for ros master then do  “export ROS_MASTER_URI=http://{ROS_MASTER_IP}:11311”
export ROS_MASTER_URI=http://192.168.0.124:11311

# if you use other ip for ToF2 then do  “export ROS_HOSTNAME={TOF2_IP}”
export ROS_HOSTNAME=192.168.0.123
roslaunch royale_ros_sample camera_driver.launch node_name:=”tof2”

exit 0

Reset the ToF sensor then ToF should publish data to /tof2/...

## MASTER

Do these command on the ROS master device:

# if you use other ip for ros master then do  “export ROS_HOSTNAME={ROS_MASTER_IP}”
export ROS_HOSTNAME=192.168.0.124

# if you use other ip for ros master then do  “export ROS_MASTER_URI=http://{ROS_MASTER_IP}:11311”
export ROS_MASTER_URI=http://192.168.0.124:11311
roscore

these code start the ROS MASTER 



on a new terminal : 

git clone -b an_ToF_emergencyBrake https://github.com/ronggurmahendra/esp-wheelchair.git
cd esp-wheelchair
python3 software/tof_potential_field.py

on a new terminal

cd esp-wheelchair
python3 software/twist_to_pwm.py

on a new terminal 

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

SIMULATION

to run the simulation 

#asuming you already done the http://wiki.ros.org/catkin/Tutorials/create_a_workspace  if not then do so 
cd catkin_ws/src
git clone https://github.com/ronggurmahendra/Robody_Sim 
cd ..
catkin_make 
source devel/setup.sh

#run gazebo world of your choice
roslaunch gazebo_ros shapes_world.launch
