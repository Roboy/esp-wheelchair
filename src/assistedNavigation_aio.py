"""
Usage:

cd esp-wheelchair
python3 src/assistedNavigation_aio.py

"""
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
from drive_mode import *

# Parameters
USE_VISUAL_POINT_CLOUD = False # USE_VISUAL_POINT_CLOUD if true will open a window that show the ToF sensor output
USE_REAR_CAMERA = False # USE_REAR_CAMERA if true will publish a data of rear camera 

USE_EMERGENCYSTOP = True # USE_EMERGENCYSTOP Will use emergency stop when distance to obstacle below the THRESHOLD_EMERGENCYSTOP
USE_SIMULATION = False

INPUT_PWM_MIN = 0 # Input PWM minimum value
INPUT_PWM_RANGE = 30 # Input PWM range value
OUTPUT_PWM_MIN = 0 # Output PWM minimum value
OUTPUT_PWM_RANGE = 20 # Output PWM range value

THRESHOLD_EMERGENCYSTOP = 0.1 # Emergency stop threshold roboy will stop if it detect a point below the thereshold

TOF_1_PITCH = 11
TOF_2_PITCH = -5

LEFT_MOTOR_TOPIC_OUTPUT = "/roboy/pinky/middleware/espchair/wheels/left"
RIGHT_MOTOR_TOPIC_OUTPUT = "/roboy/pinky/middleware/espchair/wheels/right"
ASSISTED_NAVIGATION_TOPIC_OUTPUT = '/roboy/pinky/middleware/espchair/wheels/assisted_navigation'
LEFT_MOTOR_TOPIC_INPUT = "/roboy/pinky/middleware/espchair/wheels/left/input"
RIGHT_MOTOR_TOPIC_INPUT = "/roboy/pinky/middleware/espchair/wheels/right/input"
MIN_DIST_FRONT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/front"
MIN_DIST_BACK_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/back"

# variable initialization
if(USE_VISUAL_POINT_CLOUD):
    import pcl.pcl_visualization
    viewer_front = pcl.pcl_visualization.PCLVisualizering()
    viewer_back = pcl.pcl_visualization.PCLVisualizering()
inputLinear = None
inputAngular = None
manualMode = ManualMode()
repelentMode = RepelentMode()
userInputHandler = UserInputHandler(INPUT_PWM_MIN, INPUT_PWM_RANGE)
mode = DriveMode(manualMode)

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

def modeCallBack(msg):
    """ Callback function for topic  '/roboy/pinky/middleware/espchair/wheels/mode' to change the drive mode"""
    """
    to change the mode do 
    rostopic pub /roboy/pinky/middleware/espchair/wheels/mode std_msgs/Int16 1
    """
    if(msg.data == 1):
        print("Changing Mode to Manual")
        mode.setMode(manualMode)
    elif(msg.data == 2):
        print("Changing Mode to Repelent")
        mode.setMode(repelentMode)

def repelentFieldCallBack(msg):
    """ Callback funtion for '/roboy/pinky/middleware/espchair/wheels/repelent_field' to change the tye of repellent field """
    """
    to change the function do 
    rostopic pub /roboy/pinky/middleware/espchair/wheels/repelent_field std_msgs/Int16 1
    """
    if(msg.data == 1):
        print("Changing Repelelent field to Linear")
        repelentMode.setFunction(msg.data)
    elif(msg.data == 2):
        print("Changing Repelelent field to Quadratic")
        repelentMode.setFunction(msg.data)
    elif(msg.data == 3):
        print("Changing Repelelent field to Quadratic")
        repelentMode.setFunction(msg.data)
        
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
    rotated_Pointcloud_array = Pointcloud_array

    # visualize if USE_VISUAL_POINT_CLOUD is TRUE 
    if(USE_VISUAL_POINT_CLOUD):
        if(front):
            visualizePointCloud(viewer_front, rotated_Pointcloud_array)
        else:
            visualizePointCloud(viewer_back, rotated_Pointcloud_array)
    # find the nearest point and store it at repelent Class
    minDist =  np.nanmin(rotated_Pointcloud_array[:,2])
    if(front):
        repelentMode.setDistanceFront(minDist)
    else:
        repelentMode.setDistanceBack(minDist)

def userInputCallback(msg, right):   
    """ 
    Callback funtion for user input. Takes the user input be it Twist_Teleop_Keyboard or joystick and based of variable Mode add moddification to speed
    arg right is true if the callback is for the right motor and false if the callback is for the left motor
    """
    # store user input
    userInputHandler.setUserInput(msg,right)

    # call the current Mode control function to get the adjusted output
    inputLinear,inputAngular = userInputHandler.getUserInput()
    print("inputLinear,inputAngular : ", inputLinear,inputAngular)
    outputLinear,outputAngular = mode.control(inputLinear,inputAngular)

    # if the minimum distance is within a certaun threshold then brake
    if(repelentMode.getDistanceFront() < THRESHOLD_EMERGENCYSTOP and inputLinear > 0 and USE_EMERGENCYSTOP): # this is the front ToF
        print ("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
        outputLinear = 0
    elif (repelentMode.getDistanceBack() < THRESHOLD_EMERGENCYSTOP and inputLinear < 0 and USE_EMERGENCYSTOP): # this is the back ToF
        print ("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
        outputLinear = 0
    # Check if wheelchair_emergency_stopped is defined 
    if rospy.has_param('wheelchair_emergency_stopped'):
        # check if wheelchair_emergency_stopped is TRUE
        if rospy.get_param('wheelchair_emergency_stopped'):
            return


    # publish the output to wheels 
    rospy.loginfo_throttle(5, "Publishing pwm..")
    x = max(min(outputLinear, 1.0), -1.0)
    z = max(min(outputAngular, 1.0), -1.0)
    print("linear : ", x, ", angular : ",z)
    
    if (USE_SIMULATION):
        # publish the TWIST output
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = -1*z
        assisted_navigation_pub.publish(twist)

    # publish the PWM output
    r = int(userInputHandler.translate((x + z)/2, -1, 1, -OUTPUT_PWM_RANGE, OUTPUT_PWM_RANGE ))
    l = int(userInputHandler.translate((x - z)/2, -1, 1, -OUTPUT_PWM_RANGE, OUTPUT_PWM_RANGE ))
    print("left : ", l, ", right : ",r)
    
    pub_l.publish(l)
    pub_r.publish(r)

if __name__ == "__main__":
    # init main loop
    rospy.init_node('assisted_Navigation')
    
    # initialize mode subscriber, used for changing the mode through the topic /roboy/pinky/middleware/espchair/wheels/mode
    mode_sub = rospy.Subscriber('/roboy/pinky/middleware/espchair/wheels/mode', Int16, modeCallBack)

    mode_repelent_sub = rospy.Subscriber('/roboy/pinky/middleware/espchair/wheels/repelent_field', Int16, repelentFieldCallBack)

    # initialize wheels publisher
    pub_l = rospy.Publisher(LEFT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    pub_r = rospy.Publisher(RIGHT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    
    if(USE_SIMULATION):
        # initialize TWIST publisher for simulation
        assisted_navigation_pub = rospy.Publisher(ASSISTED_NAVIGATION_TOPIC_OUTPUT, Twist, queue_size=3)
        
    # initialize subscriber for user input 
    user_input_sub_r = rospy.Subscriber(RIGHT_MOTOR_TOPIC_INPUT, Int16, userInputCallback, True)
    user_input_sub_l = rospy.Subscriber(LEFT_MOTOR_TOPIC_INPUT, Int16, userInputCallback, False)

    # initialize pointlcloud subscriber for ToF sensor
    # ToF 1 for the back hence first arg is False
    point_cloud_1_sub = rospy.Subscriber('/tof1_driver/point_cloud', PointCloud2, pointCloudCallback, (False, TOF_1_PITCH))
    # ToF 2 for the front hence first arg is True
    point_cloud_2_sub = rospy.Subscriber('/tof2_driver/point_cloud', PointCloud2, pointCloudCallback, (True, TOF_2_PITCH))

    print("publishing to /roboy/pinky/middleware/espchair/wheels/assisted_navigation. Spinning...")
    rospy.spin()
    