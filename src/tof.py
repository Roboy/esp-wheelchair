# Usage:

# cd esp-wheelchair
# python3 software/tof.py

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import pcl
import ros_numpy
import numpy as np
import pcl.pcl_visualization
from manual_control import * 
from repelent_field_control import *



# Parameters
useVisual = True
PWM_MIN = 5
PWMRANGE = 40

# variable initialization
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

sign = lambda a: (a>0) - (a<0)

def mapPwm(x, out_min, out_max):
    return x * (out_max - out_min) + out_min;

def pointCloud_to_NumpyArray(point_Cloud):
    height = point_Cloud.shape[0]
    width = point_Cloud.shape[1]
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(point_Cloud['x'], height * width)
    np_points[:, 1] = np.resize(point_Cloud['y'], height * width)
    np_points[:, 2] = np.resize(point_Cloud['z'], height * width)
    return np_points

def visualizePointCloud(viewer ,point_Cloud):
    p = pcl.PointCloud(np.array(point_Cloud, dtype=np.float32))
    viewer.AddPointCloud(p, b'scene_cloud_front', 0)
    viewer.SpinOnce()
    viewer.RemovePointCloud( b'scene_cloud_front', 0)


def point_cloud_front_callback(msg):
    # change from pointcloud2 to numpy
    pc = ros_numpy.numpify(msg)
    front_Pointcloud_array = pointCloud_to_NumpyArray(pc)
    
    if(useVisual):
        visualizePointCloud(viewer_front, front_Pointcloud_array)
    # find the nearest point
    minDist_front =  np.nanmin(front_Pointcloud_array[:,2])

def point_cloud_back_callback(msg):
    # change from pointcloud2 to numpy
    pc = ros_numpy.numpify(msg)
    back_Pointcloud_array = pointCloud_to_NumpyArray(pc)
    if(useVisual): 
        visualizePointCloud(viewer_back, back_Pointcloud_array)
    # find the nearest point
    minDist_back = np.nanmin(back_Pointcloud_array[:, 2])

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

    rospy.loginfo_throttle(5, "Publishing pwm..")
    x = max(min(outputLinear, 1.0), -1.0)
    z = max(min(outputAngular, 1.0), -1.0)

    l = (outputLinear - outputAngular) / 2.0
    r = (outputLinear + outputAngular) / 2.0

    lPwm = mapPwm(abs(l), PWM_MIN, PWMRANGE)
    rPwm = mapPwm(abs(r), PWM_MIN, PWMRANGE)
    print(" left : ", sign(l)*lPwm, ", right : ",sign(r)*rPwm)
    pub_l.publish(sign(l)*lPwm)
    pub_r.publish(sign(r)*rPwm)

# main loop
rospy.init_node('assisted_Navigation')

pub_l = rospy.Publisher("/roboy/pinky/middleware/espchair/wheels/left", Int16, queue_size=1)
pub_r = rospy.Publisher("/roboy/pinky/middleware/espchair/wheels/right", Int16, queue_size=1)

assisted_navigation_pub = rospy.Publisher('/roboy/pinky/middleware/espchair/wheels/assisted_navigation', Twist, queue_size=10)

user_input_sub = rospy.Subscriber('/cmd_vel', Twist, user_input_callback)

point_cloud_front_sub = rospy.Subscriber('/tof1_driver/point_cloud', PointCloud2, point_cloud_front_callback)
point_cloud__back_sub = rospy.Subscriber('/tof2_driver/point_cloud', PointCloud2, point_cloud_back_callback)

print("publishing to /roboy/pinky/middleware/espchair/wheels/assisted_navigation. Spinning...")
rospy.spin()