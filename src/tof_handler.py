
import rospy
from std_msgs.msg import Float64, Int16
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Twist
import ros_numpy

import pcl
import numpy as np
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

from manual_control import * 
from repelent_field_control import *
from user_input_handler import *

USE_VISUAL_POINT_CLOUD = False # USE_VISUAL_POINT_CLOUD if true will open a window that show the ToF sensor output
MIN_DIST_FRONT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/front"
MIN_DIST_BACK_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/back"

TOF_1_PITCH = 11
TOF_2_PITCH = -5

# variable initialization
if(USE_VISUAL_POINT_CLOUD):
    import pcl.pcl_visualization
    viewer_front = pcl.pcl_visualization.PCLVisualizering()
    viewer_back = pcl.pcl_visualization.PCLVisualizering()


def pointCloudToNumpyArray(point_Cloud):
    """ convert a ROS's pointcloud data structure to a numpy array"""
    height = point_Cloud.shape[0]
    width = point_Cloud.shape[1]
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(point_Cloud['x'], height * width)
    np_points[:, 1] = np.resize(point_Cloud['y'], height * width)
    np_points[:, 2] = np.resize(point_Cloud['z'], height * width)
    return np_points

def applyYRotation(point_Cloud, theta):
    """Apply a Y axis rotation matrix to pointcloud  """
    rot = [
        [ math.cos(theta), 0, math.sin(theta)],
        [ 0           , 1, 0           ],
        [-math.sin(theta), 0, math.cos(theta)]
    ]
    height = point_Cloud.shape[0]
    width = point_Cloud.shape[1]
    rotated_point_Cloud = np.zeros((height * width, 3), dtype=np.float32)
    for i in range(len(point_Cloud)):
        rotated_point_Cloud[i] = np.dot(point_Cloud[i],rot)
    return rotated_point_Cloud

def visualizePointCloud(viewer ,point_Cloud):
    """ visualize a numpy point cloud to a viewer """
    p = pcl.PointCloud(np.array(point_Cloud, dtype=np.float32))
    viewer.AddPointCloud(p, b'scene_cloud_front', 0)
    viewer.SpinOnce()
    viewer.RemovePointCloud( b'scene_cloud_front', 0)


def pointCloudCallback(msg, args):
    """ Callback function for front ToF sensor """
    # parse arg consist of boolean front and int angle  
    
    front = args[0]
    angle = args[1]

    # change from pointcloud2 to numpy
    pc = ros_numpy.numpify(msg)
    Pointcloud_array = pointCloudToNumpyArray(pc)

    # To maximize the FOV of the TOF sensor we apply a slight pitch (-5 deg for short/fat ToF and 16 deg for long ToF ) so to get the correct distance we apply a Y axis rotation matrix
    # Encountered some performance issue, Pointcloud_array shape is (38528, 3) seem to use too many resource and lagging the machine
    # rotated_Pointcloud_array = applyYRotation(Pointcloud_array,angle)

    # visualize if USE_VISUAL_POINT_CLOUD is TRUE 
    if(USE_VISUAL_POINT_CLOUD):
        if(front):
            visualizePointCloud(viewer_front, Pointcloud_array)
        else:
            visualizePointCloud(viewer_back, Pointcloud_array)

    print(Pointcloud_array.shape)
    # find the nearest point and store it at repelent Class
    minDist =  np.nanmin(Pointcloud_array[:,2])
    print(minDist)
    # publish the minimum distance to the MIN_DIST TOPIC
    if(front):
        min_dist_front_pub.publish(minDist)
        print("Minimum distance front : ", minDist)
    else:
        min_dist_back_pub.publish(minDist)
        print("Minimum distance back : ", minDist)

if __name__ == "__main__":
    # init main loop
    rospy.init_node("Tof_assistedNavigation")
    
    # initialize pointlcloud subscriber for ToF sensor
    # ToF 1 for the back hence first arg is False
    point_cloud_1_sub = rospy.Subscriber('/tof1_driver/point_cloud', PointCloud2, pointCloudCallback, (False, TOF_1_PITCH))
    # ToF 2 for the front hence first arg is True
    point_cloud_2_sub = rospy.Subscriber('/tof2_driver/point_cloud', PointCloud2, pointCloudCallback, (True, TOF_2_PITCH))

    min_dist_front_pub = rospy.Publisher(MIN_DIST_FRONT_TOPIC, Float64, queue_size=3)
    min_dist_back_pub = rospy.Publisher(MIN_DIST_BACK_TOPIC, Float64, queue_size=3)

    print("publishing to " + MIN_DIST_FRONT_TOPIC + " and " + MIN_DIST_BACK_TOPIC + " Spinning...")
    rospy.spin()
    