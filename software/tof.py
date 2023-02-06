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
from manual import * 

useVisual = False
emergencyStopThreshold = 0.1

viewer_front = pcl.pcl_visualization.PCLVisualizering()
viewer_back = pcl.pcl_visualization.PCLVisualizering()

def getNearestDistance(points):
    min = 9999
    for i in range(len(points)):
        if(min > points[i][2]):
            min = points[i][2]
    return min
    
def point_cloud_front_callback(msg):
    # change from pointcloud2 to numpy
    pc = ros_numpy.numpify(msg)
    height = pc.shape[0]
    width = pc.shape[1]
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)

    p = pcl.PointCloud(np.array(np_points, dtype=np.float32))
    viewer_front.AddPointCloud(p, b'scene_cloud_front', 0)
    viewer_front.SpinOnce()
    viewer_front.RemovePointCloud( b'scene_cloud_front', 0)
    # find the nearest point
    global minDist_front
    minDist_front = getNearestDistance(np_points)

def point_cloud_back_callback(msg):
    # change from pointcloud2 to numpy
    pc = ros_numpy.numpify(msg)
    height = pc.shape[0]
    width = pc.shape[1]
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)

    p = pcl.PointCloud(np.array(np_points, dtype=np.float32))
    viewer_back.AddPointCloud(p, b'scene_cloud_back', 0)
    viewer_back.SpinOnce()
    viewer_back.RemovePointCloud( b'scene_cloud_back', 0)
    # find the nearest point
    global minDist_back
    minDist_back = getNearestDistance(np_points)

def user_input_callback(msg):
    global inputLinear, inputAngular
    inputLinear = msg.linear.x
    inputAngular = msg.angular.z
    outputAngular = inputAngular
    outputLinear = inputLinear

# def main():
rospy.init_node('assisted_Navigation')

assisted_navigation_pub = rospy.Publisher('/roboy/pinky/middleware/espchair/wheels/assisted_navigation', Twist, queue_size=10)
user_input_sub = rospy.Subscriber('/cmd_vel', Twist, user_input_callback)
# point_cloud_sub = rospy.Subscriber('/royale_camera_driver/point_cloud', PointCloud2, point_cloud_callback)

point_cloud_front_sub = rospy.Subscriber('/tof1_driver/point_cloud', PointCloud2, point_cloud_front_callback)
point_cloud__back_sub = rospy.Subscriber('/tof2_driver/point_cloud', PointCloud2, point_cloud_back_callback)

rospy.loginfo("publishing to /roboy/pinky/middleware/espchair/wheels/assisted_navigation. Spinning...")

manualMode = ManualMode()
    
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    if(minDist_front < emergencyStopThreshold and inputLinear > 0): # asume that this is the front ToF
        rospy.loginfo ("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
        outputLinear = 0
    
    elif (minDist_back < emergencyStopThreshold and inputLinear < 0): # asume that this is the back ToF
        rospy.loginfo ("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
        outputLinear = 0


    outputLinear,outputAngular = manualMode.control(inputLinear,inputAngular)
    rospy.loginfo("outputLinear : ", outputLinear, "outputAngular  : ",  outputAngular)
    
    twist = Twist()
    twist.linear.x = outputLinear
    twist.angular.z = outputAngular
    assisted_navigation_pub.publish(twist)
