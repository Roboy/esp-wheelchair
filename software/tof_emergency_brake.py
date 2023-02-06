# Usage:

# cd esp-wheelchair
# python3 software/tof_emergency_brake.py

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import pcl
import ros_numpy
import numpy as np
import pcl.pcl_visualization

emergencyStopThreshold = 0.1
viewer_front = pcl.pcl_visualization.PCLVisualizering()
    
viewer_back = pcl.pcl_visualization.PCLVisualizering()
    
visualDone = False

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
    viewer_front.AddPointCloud(p, b'scene_cloud', 0)
    viewer_front.SpinOnce()
    viewer_front.RemovePointCloud( b'scene_cloud', 0)
    # find the nearest point
    global minDist_front
    minDist_front = 99999

    for i in range(len(np_points)):
        if(minDist_front > np_points[i][2]):
            minDist_front = np_points[i][2]

    rospy.loginfo("min front: %f", minDist_front )
    if minDist_front < emergencyStopThreshold and inputLinear > 0: # emergency stop
        outputspeed = 0.0
    
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
    viewer_back.AddPointCloud(p, b'scene_cloud', 0)
    viewer_back.SpinOnce()
    viewer_back.RemovePointCloud( b'scene_cloud', 0)
    # find the nearest point
    global minDist_back
    minDist_back = 99999

    for i in range(len(np_points)):
        if(minDist_back > np_points[i][2]):
            minDist_back = np_points[i][2]

    rospy.loginfo("min back: %f", minDist_back )
    if minDist_back < emergencyStopThreshold and inputLinear < 0: # emergency stop
        outputspeed = 0.0
    


def user_input_callback(msg):
    global inputLinear, inputAngular, outputAngular
    inputLinear = msg.linear.x
    inputAngular = msg.angular.z
    outputAngular = inputAngular
    outputLinear = inputLinear
    if(minDist_front < emergencyStopThreshold and inputLinear > 0): # asume that this is the front ToF
        rospy.loginfo ("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
        outputLinear = 0
    
    elif (minDist_back < emergencyStopThreshold and inputLinear < 0): # asume that this is the back ToF
        rospy.loginfo ("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
        outputLinear = 0

    twist = Twist()
    print("outputLinear : ", outputLinear, "outputAngular  : ",  outputAngular)
    twist.linear.x = outputLinear
    twist.angular.z = outputAngular
    emergency_brake_pub.publish(twist)


    

# def main():
rospy.init_node('tof_emergency_brake')

emergency_brake_pub = rospy.Publisher('/roboy/pinky/middleware/espchair/wheels/emergency_brake', Twist, queue_size=10)
user_input_sub = rospy.Subscriber('/cmd_vel', Twist, user_input_callback)
# point_cloud_sub = rospy.Subscriber('/royale_camera_driver/point_cloud', PointCloud2, point_cloud_callback)

point_cloud_front_sub = rospy.Subscriber('/tof1_driver/point_cloud', PointCloud2, point_cloud_front_callback)
point_cloud__back_sub = rospy.Subscriber('/tof2_driver/point_cloud', PointCloud2, point_cloud_back_callback)


rospy.loginfo("publishing to /roboy/pinky/middleware/espchair/wheels/emergency_brake. Spinning...")
rospy.spin()
