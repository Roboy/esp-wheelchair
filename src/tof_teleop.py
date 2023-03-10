"""
Usage:

cd esp-wheelchair
python3 src/tof_teleop.py

"""
import rospy
from std_msgs.msg import Float64, Int16
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import pcl
import ros_numpy
import numpy as np
import pcl.pcl_visualization
from manual_control import * 
from repelent_field_control import *

# Parameters
USEVISUAL = True # USEVISUAL if true will open a window that show the ToF sensor output
PWM_MIN = 5 # PWM minimum value
PWMRANGE = 40 # PWM range value
EMERGENCYSTOPTHRESHOLD = 0.1 # emergency stop threshold roboy will stop if it detect a point below the thereshold


# variable initialization
if(USEVISUAL):
    viewer_front = pcl.pcl_visualization.PCLVisualizering()
    viewer_back = pcl.pcl_visualization.PCLVisualizering()
inputLinear = None
inputAngular = None
manualMode = ManualMode()
repelentMode = RepelentMode()
Mode = manualMode

sign = lambda a: (a>0) - (a<0)

def mapPwm(x, out_min, out_max):
    """Map the x value 0.0 - 1.0 to out_min to out_max"""
    return x * (out_max - out_min) + out_min

def pointCloudToNumpyArray(point_Cloud):
    """ convert a ROS's pointcloud data structure to a numpy array"""
    height = point_Cloud.shape[0]
    width = point_Cloud.shape[1]
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(point_Cloud['x'], height * width)
    np_points[:, 1] = np.resize(point_Cloud['y'], height * width)
    np_points[:, 2] = np.resize(point_Cloud['z'], height * width)
    return np_points

def visualizePointCloud(viewer ,point_Cloud):
    """ visualize a numpy point cloud to a viewer """
    p = pcl.PointCloud(np.array(point_Cloud, dtype=np.float32))
    viewer.AddPointCloud(p, b'scene_cloud_front', 0)
    viewer.SpinOnce()
    viewer.RemovePointCloud( b'scene_cloud_front', 0)

def pointCloudCallback(msg,front):
    """ Callback function for front ToF sensor """
    # change from pointcloud2 to numpy
    pc = ros_numpy.numpify(msg)
    Pointcloud_array = pointCloudToNumpyArray(pc)
    # visualize if USEVISUAL is TRUE 
    if(USEVISUAL):
        if(front):
            visualizePointCloud(viewer_front, Pointcloud_array)
        else:
            visualizePointCloud(viewer_back, Pointcloud_array)
    # find the nearest point and store it at repelent Class
    minDist =  np.nanmin(Pointcloud_array[:,2])
    if(front):
        repelentMode.setDistanceFront(minDist)
    else:
        repelentMode.setDistanceBack(minDist)

def userInputCallback(msg):   
    """ Callback funtion for user input. Takes the user input be it Twist_Teleop_Keyboard or joystick and based of variable Mode add moddification to speed """
    # store user input
    inputLinear = msg.linear.x
    inputAngular = msg.angular.z

    # call the current Mode control function
    outputLinear,outputAngular = Mode.control(inputLinear,inputAngular)

    # if the minimum distance is within a certaun threshold then brake
    if(repelentMode.getDistanceFront() < EMERGENCYSTOPTHRESHOLD and inputLinear > 0): # this is the front ToF
        print ("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
        outputLinear = 0
    elif (repelentMode.getDistanceBack() < EMERGENCYSTOPTHRESHOLD and inputLinear < 0): # this is the back ToF
        print ("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
        outputLinear = 0
    # Check if wheelchair_emergency_stopped is defined 
    if rospy.has_param('wheelchair_emergency_stopped'):
        # check if wheelchair_emergency_stopped is TRUE
        if rospy.get_param('wheelchair_emergency_stopped'):
            return

    # publish the TWIST output to simulation
    twist = Twist()
    twist.linear.x = outputLinear
    twist.angular.z = outputAngular
    assisted_navigation_pub.publish(twist)

    # publish the output to wheels 
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

if __name__ == "__main__":
    # init main loop
    rospy.init_node('assisted_Navigation')
    
    # initialize mode with manual mode
    Mode = manualMode
    
    # initialize wheels publisher
    pub_l = rospy.Publisher("/roboy/pinky/middleware/espchair/wheels/left", Int16, queue_size=1)
    pub_r = rospy.Publisher("/roboy/pinky/middleware/espchair/wheels/right", Int16, queue_size=1)
    
    # initialize TWIST publisher for simulation
    assisted_navigation_pub = rospy.Publisher('/roboy/pinky/middleware/espchair/wheels/assisted_navigation', Twist, queue_size=10)
    
    # initialize TWIST subscriber for user input 
    user_input_sub = rospy.Subscriber('/cmd_vel', Twist, userInputCallback)
    
    # initialize pointlcloud subscriber for ToF sensor
    point_cloud_front_sub = rospy.Subscriber('/tof1_driver/point_cloud', PointCloud2, pointCloudCallback, True)
    point_cloud__back_sub = rospy.Subscriber('/tof2_driver/point_cloud', PointCloud2, pointCloudCallback, False)
    
    print("publishing to /roboy/pinky/middleware/espchair/wheels/assisted_navigation. Spinning...")
    rospy.spin()
    