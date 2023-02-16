# Usage:

# cd esp-wheelchair
# python3 software/tof.py

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import pcl
import ros_numpy
import numpy as np
import pcl.pcl_visualization
from manual_control import * 
from repelent_field_control import *
useVisual = False

emergencyStopThreshold = 0.1
if(useVisual):
    viewer_front = pcl.pcl_visualization.PCLVisualizering()
    viewer_back = pcl.pcl_visualization.PCLVisualizering()
minDist_front = 9999
minDist_back = 9999
inputLinear = None
inputAngular = None
manualMode = ManualMode()
repelentMode = RepelentMode()

def pointCloud_to_NumpyArray(point_Cloud):
    height = point_Cloud.shape[0]
    width = point_Cloud.shape[1]
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(point_Cloud['x'], height * width)
    np_points[:, 1] = np.resize(point_Cloud['y'], height * width)
    np_points[:, 2] = np.resize(point_Cloud['z'], height * width)
    return np_points

def visualizePointCloud(viewer ,point_Cloud):
    if(useVisual):
        p = pcl.PointCloud(np.array(point_Cloud, dtype=np.float32))
        viewer.AddPointCloud(p, b'scene_cloud_front', 0)
        viewer.SpinOnce()
        viewer.RemovePointCloud( b'scene_cloud_front', 0)


def point_cloud_front_callback(msg):
    # change from pointcloud2 to numpy
    pc = ros_numpy.numpify(msg)
    front_Pointcloud_array = pointCloud_to_NumpyArray(pc)
    visualizePointCloud(viewer_front, front_Pointcloud_array)
    # find the nearest point
    minDist_front =  np.amin(front_Pointcloud_array[:, 2])

def point_cloud_back_callback(msg):
    # change from pointcloud2 to numpy
    pc = ros_numpy.numpify(msg)
    back_Pointcloud_array = pointCloud_to_NumpyArray(pc)
    visualizePointCloud(viewer_back, back_Pointcloud_array)
    # find the nearest point
    minDist_back = np.amin(back_Pointcloud_array[:, 2])

def user_input_callback(msg):   
    inputLinear = msg.linear.x
    inputAngular = msg.angular.z

    # manual mode
    outputLinear,outputAngular = manualMode.control(inputLinear,inputAngular)

    # repelent field 
    # outputLinear,outputAngular = repelentMode.control(inputLinear,inputAngular,minDist_front,minDist_back)

    if(minDist_front < emergencyStopThreshold and inputLinear > 0): # asume that this is the front ToF
        print ("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
        outputLinear = 0
    elif (minDist_back < emergencyStopThreshold and inputLinear < 0): # asume that this is the back ToF
        print ("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
        outputLinear = 0

    twist = Twist()
    twist.linear.x = outputLinear
    twist.angular.z = outputAngular
    assisted_navigation_pub.publish(twist)

# main loop
rospy.init_node('assisted_Navigation')

assisted_navigation_pub = rospy.Publisher('/roboy/pinky/middleware/espchair/wheels/assisted_navigation', Twist, queue_size=10)
user_input_sub = rospy.Subscriber('/cmd_vel', Twist, user_input_callback)

point_cloud_front_sub = rospy.Subscriber('/tof1_driver/point_cloud', PointCloud2, point_cloud_front_callback)
point_cloud__back_sub = rospy.Subscriber('/tof2_driver/point_cloud', PointCloud2, point_cloud_back_callback)

print("publishing to /roboy/pinky/middleware/espchair/wheels/assisted_navigation. Spinning...")
rospy.spin()