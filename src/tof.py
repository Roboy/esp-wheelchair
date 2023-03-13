"""
Usage:

cd esp-wheelchair
python3 src/tof.py

"""
import rospy
from std_msgs.msg import Float64, Int16
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import pcl
import ros_numpy
import numpy as np
from manual_control import * 
from repelent_field_control import *
from user_input_handler import *

# Parameters
USEVISUAL = False # USEVISUAL if true will open a window that show the ToF sensor output
INPUT_PWM_MIN = 0 # Input PWM minimum value
INPUT_PWM_RANGE = 30 # Input PWM range value
OUTPUT_PWM_MIN = 0 # Output PWM minimum value
OUTPUT_PWM_RANGE = 0 # Output PWM range value
USE_EMERGENCYSTOP = 1 # Will use emergency stop
EMERGENCYSTOPTHRESHOLD = 0.1 # Emergency stop threshold roboy will stop if it detect a point below the thereshold

# variable initialization
if(USEVISUAL):
    import pcl.pcl_visualization
    viewer_front = pcl.pcl_visualization.PCLVisualizering()
    viewer_back = pcl.pcl_visualization.PCLVisualizering()
inputLinear = None
inputAngular = None
manualMode = ManualMode()
repelentMode = RepelentMode()
userInputHandler = UserInputHandler(INPUT_PWM_MIN, INPUT_PWM_RANGE)
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

def modeCallBack(msg):
    """ Callback function for topic  '/roboy/pinky/middleware/espchair/wheels/mode' to change the drive mode"""
    if(msg.data == 1):
        print("Changing Mode to Manual")
        Mode = manualMode
    elif(msg.data == 2):
        print("Changing Mode to Repelent")
        Mode = repelentMode

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

def userInputCallback(msg, right):   
    """ 
    Callback funtion for user input. Takes the user input be it Twist_Teleop_Keyboard or joystick and based of variable Mode add moddification to speed
    arg right is true if the callback is for the right motor and false if the callback is for the left motor
    """
    # store user input
    userInputHandler.setUserInput(msg,right)

    # call the current Mode control function to get the adjusted output
    inputLinear,inputAngular = userInputHandler.getUserInput()
    # print("inputLinear,inputAngular : ", inputLinear,inputAngular)
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


    # publish the output to wheels 
    rospy.loginfo_throttle(5, "Publishing pwm..")
    x = max(min(outputLinear, 1.0), -1.0)
    z = max(min(outputAngular, 1.0), -1.0)
    print("linear : ", x, ", angular : ",z)
    
    # publish the TWIST output
    twist = Twist()
    twist.linear.x = x
    twist.angular.z = -1*z
    assisted_navigation_pub.publish(twist)

    # publish the PWM output
    r = userInputHandler.translate((x + z)/2, -1, 1, -OUTPUT_PWM_RANGE, OUTPUT_PWM_RANGE )
    l = userInputHandler.translate((x - z)/2, -1, 1, -OUTPUT_PWM_RANGE, OUTPUT_PWM_RANGE )
    # print("left : ", l, ", right : ",r)
    
    pub_l.publish(l)
    pub_r.publish(r)

if __name__ == "__main__":
    # init main loop
    rospy.init_node('assisted_Navigation')
    
    # initialize mode subscriber, used for changing the mode through the topic /roboy/pinky/middleware/espchair/wheels/mode
    mode_sub = rospy.Subscriber('/roboy/pinky/middleware/espchair/wheels/mode', Int16, modeCallBack)

    # initialize wheels publisher
    pub_l = rospy.Publisher("/roboy/pinky/middleware/espchair/wheels/left/adjusted", Int16, queue_size=10)
    pub_r = rospy.Publisher("/roboy/pinky/middleware/espchair/wheels/right/adjusted", Int16, queue_size=10)
    
    # initialize TWIST publisher for simulation
    assisted_navigation_pub = rospy.Publisher('/roboy/pinky/middleware/espchair/wheels/assisted_navigation', Twist, queue_size=10)
    
    # initialize subscriber for user input 
    user_input_sub_r = rospy.Subscriber('/roboy/pinky/middleware/espchair/wheels/right', Int16, userInputCallback, True)
    user_input_sub_l = rospy.Subscriber('/roboy/pinky/middleware/espchair/wheels/left', Int16, userInputCallback, False)

    # initialize pointlcloud subscriber for ToF sensor
    point_cloud_front_sub = rospy.Subscriber('/tof1_driver/point_cloud', PointCloud2, pointCloudCallback, True)
    point_cloud_back_sub = rospy.Subscriber('/tof2_driver/point_cloud', PointCloud2, pointCloudCallback, False)
    
    print("publishing to /roboy/pinky/middleware/espchair/wheels/assisted_navigation. Spinning...")
    rospy.spin()
    