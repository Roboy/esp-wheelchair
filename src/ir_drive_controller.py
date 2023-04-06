"""
Usage:

python ir_drive_controller.py
python ir_publisher.py

and if you are using teleop keyboard use 

python twist_to_pwm.py
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

"""
import rospy
from std_msgs.msg import Int16, Int16MultiArray
from geometry_msgs.msg import Twist

import numpy as np

from user_input_handler import UserInputHandler
from IR_State import IRState

# Parameters
USE_EMERGENCYSTOP = True # USE_EMERGENCYSTOP Will use emergency stop when distance to obstacle below the THRESHOLD_EMERGENCYSTOP
USE_SIMULATION = True

INPUT_PWM_MIN = 0 # Input PWM minimum value
INPUT_PWM_RANGE = 30 # Input PWM range value
OUTPUT_PWM_MIN = 0 # Output PWM minimum value
OUTPUT_PWM_RANGE = 20 # Output PWM range value

LEFT_MOTOR_TOPIC_OUTPUT = "/roboy/pinky/middleware/espchair/wheels/left"
RIGHT_MOTOR_TOPIC_OUTPUT = "/roboy/pinky/middleware/espchair/wheels/right"

LEFT_MOTOR_TOPIC_INPUT = "/roboy/pinky/middleware/espchair/wheels/left/input"
RIGHT_MOTOR_TOPIC_INPUT = "/roboy/pinky/middleware/espchair/wheels/right/input"

# Publish TWIST output mainly for simulation 
ASSISTED_NAVIGATION_TOPIC_OUTPUT = '/roboy/pinky/middleware/espchair/wheels/assisted_navigation'

# Set up publishers for each distance topic
IR_TOPIC = "/roboy/pinky/sensing/distance"
IR_FRONT_RIGHT_ID = 0
IR_FRONT_LEFT_ID = 1
IR_BACK_RIGHT_ID = 2
IR_BACK_LEFT_ID = 3

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
    userInputHandler.setUserInput(msg, right)

    # call the current Mode control function to get the adjusted output
    inputLinear,inputAngular = userInputHandler.getUserInput()
    print("inputLinear, inputAngular : ", inputLinear,inputAngular)
    outputLinear, outputAngular = inputLinear, inputAngular 
    # if the minimum distance is within a certaun threshold then brake
    if((irState.get(IRState._FRONT_RIGHT) or irState.get(IRState._FRONT_LEFT)) and inputLinear > 0 and USE_EMERGENCYSTOP): # check if it about to collide in the front
        print ("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
        outputLinear = 0
    elif ((irState.get(IRState._BACK_RIGHT) or irState.get(IRState._BACK_RIGHT)) and inputLinear < 0 and USE_EMERGENCYSTOP): # check if it about to collide in the back
        print ("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
        outputLinear = 0

    # publish the output to wheels 
    rospy.loginfo_throttle(5, "Publishing pwm..")
    x = max(min(outputLinear, 1.0), -1.0)
    z = max(min(outputAngular, 1.0), -1.0)
    print("linear : ", x, ", angular : ",z)
    
    # publish TWIST output to simulaiton if USE_SIMULATION is true
    if (USE_SIMULATION):
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = -1*z
        assisted_navigation_pub.publish(twist)

    # publish the PWM output to the motor
    r = int(userInputHandler.translate((x + z)/2, -1, 1, -OUTPUT_PWM_RANGE, OUTPUT_PWM_RANGE ))
    l = int(userInputHandler.translate((x - z)/2, -1, 1, -OUTPUT_PWM_RANGE, OUTPUT_PWM_RANGE ))
    print("left : ", l, ", right : ",r)
    
    pub_motor_l.publish(l)
    pub_motor_r.publish(r)

def irSensorCallback(msg):
    irState.set(msg.data)
    return
    
if __name__ == "__main__":
    # init main loop
    rospy.init_node('Ir_AssistedNavigation_main')
    
    # initialize wheels publisher
    pub_motor_l = rospy.Publisher(LEFT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    pub_motor_r = rospy.Publisher(RIGHT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    
    if(USE_SIMULATION):
        # initialize TWIST publisher for simulation
        assisted_navigation_pub = rospy.Publisher(ASSISTED_NAVIGATION_TOPIC_OUTPUT, Twist, queue_size=3)
        
    # initialize subscriber for user input 
    user_input_sub_r = rospy.Subscriber(RIGHT_MOTOR_TOPIC_INPUT, Int16, userInputCallback, True)
    user_input_sub_l = rospy.Subscriber(LEFT_MOTOR_TOPIC_INPUT, Int16, userInputCallback, False)

    #Initialize subscriber for Ir sensor
    ir_sub = rospy.Subscriber(IR_TOPIC, Int16MultiArray, irSensorCallback)

    print("publishing to /roboy/pinky/middleware/espchair/wheels/assisted_navigation. Spinning...")
    rospy.spin()
    