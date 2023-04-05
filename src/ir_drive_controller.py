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

from user_input_handler import UserInputHandler
from IR_State import IRState
# Parameters
USE_EMERGENCYSTOP = True # USE_EMERGENCYSTOP Will use emergency stop when distance to obstacle below the THRESHOLD_EMERGENCYSTOP
USE_SIMULATION = True

INPUT_PWM_MIN = 0 # Input PWM minimum value
INPUT_PWM_RANGE = 30 # Input PWM range value
OUTPUT_PWM_MIN = 0 # Output PWM minimum value
OUTPUT_PWM_RANGE = 20 # Output PWM range value

THRESHOLD_EMERGENCYSTOP = 0.1 # Emergency stop threshold roboy will stop if it detect a point below the thereshold

LEFT_MOTOR_TOPIC_OUTPUT = "/roboy/pinky/middleware/espchair/wheels/left"
RIGHT_MOTOR_TOPIC_OUTPUT = "/roboy/pinky/middleware/espchair/wheels/right"

LEFT_MOTOR_TOPIC_INPUT = "/roboy/pinky/middleware/espchair/wheels/left/input"
RIGHT_MOTOR_TOPIC_INPUT = "/roboy/pinky/middleware/espchair/wheels/right/input"

ASSISTED_NAVIGATION_TOPIC_OUTPUT = '/roboy/pinky/middleware/espchair/wheels/assisted_navigation'

# Set up publishers for each distance topic
DIST_FRONT_RIGHT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/front_right"
DIST_FRONT_LEFT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/front_left"
DIST_BACK_RIGHT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/back_right"
DIST_BACK_LEFT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/back_left"

inputLinear = None
inputAngular = None
userInputHandler = UserInputHandler(INPUT_PWM_MIN, INPUT_PWM_RANGE)
irState = IRState()

sign = lambda a: (a>0) - (a<0)

def mapPwm(x, out_min, out_max):
    """Map the x value 0.0 - 1.0 to out_min to out_max"""
    return x * (out_max - out_min) + out_min

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
    outputLinear, outputAngular = inputLinear, inputAngular 
    # if the minimum distance is within a certaun threshold then brake
    if((irState.get(IRState._FRONT_RIGHT) or irState.get(IRState._FRONT_LEFT)) and inputLinear > 0 and USE_EMERGENCYSTOP): # this is the front ToF
        print ("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
        outputLinear = 0
    elif ((irState.get(IRState._BACK_RIGHT) or irState.get(IRState._BACK_RIGHT)) and inputLinear < 0 and USE_EMERGENCYSTOP): # this is the back ToF
        print ("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
        outputLinear = 0

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

def irSensorCallback(msg, pos):
    irState.set(pos,msg.data)
    return
    
if __name__ == "__main__":
    # init main loop
    rospy.init_node('Ir_AssistedNavigation_main')
    
    # initialize wheels publisher
    pub_l = rospy.Publisher(LEFT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    pub_r = rospy.Publisher(RIGHT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    
    if(USE_SIMULATION):
        # initialize TWIST publisher for simulation
        assisted_navigation_pub = rospy.Publisher(ASSISTED_NAVIGATION_TOPIC_OUTPUT, Twist, queue_size=3)
        
    # initialize subscriber for user input 
    user_input_sub_r = rospy.Subscriber(RIGHT_MOTOR_TOPIC_INPUT, Int16, userInputCallback, True)
    user_input_sub_l = rospy.Subscriber(LEFT_MOTOR_TOPIC_INPUT, Int16, userInputCallback, False)

    #Initialize subscriber for Ir sensor topic
    front_right_sub = rospy.Subscriber(DIST_FRONT_RIGHT_TOPIC, Int16, irSensorCallback, 1)
    front_left_sub = rospy.Subscriber(DIST_FRONT_LEFT_TOPIC, Int16, irSensorCallback, 2)
    back_right_sub = rospy.Subscriber(DIST_BACK_RIGHT_TOPIC, Int16, irSensorCallback, 3)
    back_left_sub = rospy.Subscriber(DIST_BACK_LEFT_TOPIC, Int16, irSensorCallback, 4)

    print("publishing to /roboy/pinky/middleware/espchair/wheels/assisted_navigation. Spinning...")
    rospy.spin()
    