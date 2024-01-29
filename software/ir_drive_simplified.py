#!/usr/bin/python3

# this is a simplified version of ir_drive_controller it didnt change the PWM input into a linear and angular insted just forward it after checking if its going to collide
"""
Usage:

python ir_sensor_publisher.py
python ir_drive_controller.py

and if you are using teleop keyboard use 

python twist_to_pwm.py
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

"""
import rospy
from std_msgs.msg import Int16, Int16MultiArray
from geometry_msgs.msg import Twist

from numpy import sign
import sys
import signal

# Parameters
rospy.set_param('USE_ASSISTED_DRIVING', False) # USE_EMERGENCYSTOP Will use emergency stop when distance to obstacle below the THRESHOLD_EMERGENCYSTOP
rospy.set_param("wheels_boost_factor", 1.2)


INPUT_PWM_MIN = 0 # Input PWM minimum value
INPUT_PWM_RANGE = 30 # Input PWM range value
OUTPUT_PWM_MIN = 0 # Output PWM minimum value
OUTPUT_PWM_RANGE = 20 # Output PWM range value
MAX_BOOST = 2.5

PWM_MAX = 30

LEFT_MOTOR_TOPIC_OUTPUT = "/roboy/pinky/middleware/espchair/wheels/left"
RIGHT_MOTOR_TOPIC_OUTPUT = "/roboy/pinky/middleware/espchair/wheels/right"

LEFT_MOTOR_TOPIC_INPUT = "/operator/wheels/left" #"/roboy/pinky/middleware/espchair/wheels/left/input"
RIGHT_MOTOR_TOPIC_INPUT = "/operator/wheels/right" #"/roboy/pinky/middleware/espchair/wheels/right/input"

# Publish TWIST output mainly for simulation 
ASSISTED_NAVIGATION_TOPIC_OUTPUT = '/roboy/pinky/middleware/espchair/wheels/assisted_navigation'

# Set up publishers for each distance topic
IR_TOPIC = "/roboy/pinky/sensing/distance"
# these are the index of the infrared sensor published at IR_TOPIC
IR_FRONT_RIGHT_ID = 0
IR_FRONT_LEFT_ID = 1
IR_BACK_RIGHT_ID = 2
IR_BACK_LEFT_ID = 3

global last_cmds
last_cmds = [0,0]

# place holder to store ir sensor data
class IRState:
    """ Manual Mode no modification between user input and output """
    _FRONT_RIGHT_ID = 0
    _FRONT_LEFT_ID = 1
    _BACK_RIGHT_ID = 2
    _BACK_LEFT_ID = 3
    def __init__(self):
        self.ir_sensor = [1,1,1,1]
        return
    
    def set(self, input):
        self.ir_sensor = input
        return 

    def get(self,pos):
        return self.ir_sensor[pos]
    
irState = IRState()

# IR sensor topic callback function
def irSensorCallback(msg):
    irState.set(msg.data)
    return

def publish_cmd(pub, msg, right):
    global boost_factor
    msg.data = int(msg.data*boost_factor)
    idx = 0 if right else 1
    
    if msg.data == 0:
        if last_cmds[idx] == 0:
            pub.publish(0)
        else:
            rospy.loginfo("slowing down")
            for i in range(abs(last_cmds[idx]), 0, -1):
                pub.publish(sign(last_cmds[idx]) * i)
                #rospy.sleep(0.05)
                rospy.sleep(0.05)
        last_cmds[idx] = 0 #sign(last_cmds[idx]) * i
    elif last_cmds[idx] == 0:
        rospy.loginfo("slowing down 0.5")
        for i in range(0, abs(last_cmds[idx]), 1):
            pub.publish(sign(msg.data)*i)
            rospy.sleep(0.5)
        last_cmds[idx] = msg.data #sign(msg.data) * i
    else:
        pub.publish(msg.data)
        last_cmds[idx] = msg.data
    # rospy.loginfo_throttle(1,last_cmds[idx])
    # pub.publish(msg.data)

    
def userInputCallbackRight(msg):
    global last_cmds, ad
    
    turning = True if sign(last_cmds[0]) != sign(last_cmds[1]) else False
    if ad and not turning:
        if((not irState.get(irState._FRONT_RIGHT_ID) or not irState.get(irState._FRONT_LEFT_ID)) and msg.data > 0): # check if it about to collide in the front
            rospy.loginfo("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
            msg.data = 0
        elif ((not irState.get(irState._BACK_RIGHT_ID) or not irState.get(irState._BACK_LEFT_ID)) and msg.data < 0): # check if it about to collide in the back
            rospy.loginfo("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
            msg.data = 0

    publish_cmd(pub_motor_r, msg, True)

def userInputCallbackLeft(msg):
    global last_cmds, ad
    # ad = rospy.get_param('USE_ASSISTED_DRIVING')
    turning = True if sign(last_cmds[0]) != sign(last_cmds[1]) else False

    if ad and not turning:
        if((not irState.get(irState._FRONT_RIGHT_ID) or not irState.get(irState._FRONT_LEFT_ID)) and msg.data > 0): # check if it about to collide in the front
            rospy.loginfo("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
            msg.data = 0
        elif ((not irState.get(irState._BACK_RIGHT_ID) or not irState.get(irState._BACK_LEFT_ID)) and msg.data < 0): # check if it about to collide in the back
            rospy.loginfo("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
            msg.data = 0

    publish_cmd(pub_motor_l, msg, False)

def signal_handler(sig, frame):
    # Perform cleanup and shutdown routines here
    print('Ctrl+C detected, shutting down...')
    sys.exit(0)


if __name__ == "__main__":
    # init main loop
    rospy.init_node('assisted_drive_controller')
    
    # initialize wheels publisher
    pub_motor_l = rospy.Publisher(LEFT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    pub_motor_r = rospy.Publisher(RIGHT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    
    # initialize subscriber for user input 
    user_input_sub_r = rospy.Subscriber(RIGHT_MOTOR_TOPIC_INPUT, Int16, userInputCallbackRight)
    user_input_sub_l = rospy.Subscriber(LEFT_MOTOR_TOPIC_INPUT, Int16, userInputCallbackLeft)

    #Initialize subscriber for Ir sensor
    ir_sub = rospy.Subscriber(IR_TOPIC, Int16MultiArray, irSensorCallback)

    rospy.loginfo("Assisted navigation node started. Publishing to /roboy/pinky/middleware/espchair/wheels/left & right. Spinning...")
    rate = rospy.Rate(100)

    signal.signal(signal.SIGINT, signal_handler)

    global ad, boost_factor
    while not rospy.is_shutdown():
        ad = rospy.get_param('USE_ASSISTED_DRIVING')
        boost = rospy.get_param("wheels_boost_factor")
        if abs(boost) <= MAX_BOOST:
            boost_factor = abs(boost)           
        else:
            boost_factor = MAX_BOOST
            rospy.logwarn(f"Boost factor set too high, using maximum possible: {boost_factor}")
        rospy.loginfo_throttle(10, f"Using boost factor {boost_factor}")
        # rate.sleep()
    # rospy.spin()
