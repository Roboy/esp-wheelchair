# Assisted Navigation

## How to Run 
Setup ToF according to this documentation : https://devanthro.atlassian.net/wiki/spaces/WS2223/pages/2747269188/AN+Software

Or 

Use Bagfile provided in this [link](https://drive.google.com/drive/folders/15IJ5Bk0Abo6tRJbfVam8PI72EpSFbLXI?usp=sharing)

to execute use 
```
rosbag play -l ToFSensor_sample.bag /royale_camera_driver/pointcloud:=/tof1_driver/point_cloud &
rosbag play -l ToFSensor_sample2.bag /royale_camera_driver/pointcloud:=/tof2_driver/point_cloud
```

And the to run the assisted navigation node
```
# if you havent have any rosmaster running
roscore &

python3 ../src/tof.py & 
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

```
