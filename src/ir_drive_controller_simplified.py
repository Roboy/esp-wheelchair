
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

# Parameters
USE_EMERGENCYSTOP = True # USE_EMERGENCYSTOP Will use emergency stop when distance to obstacle below the THRESHOLD_EMERGENCYSTOP

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
# these are the index of the infrared sensor published at IR_TOPIC
IR_FRONT_RIGHT_ID = 0
IR_FRONT_LEFT_ID = 1
IR_BACK_RIGHT_ID = 2
IR_BACK_LEFT_ID = 3

# place holder to store ir sensor data
class IRState:
    """ Manual Mode no modification between user input and output """
    _FRONT_RIGHT_ID = 0
    _FRONT_LEFT_ID = 1
    _BACK_RIGHT_ID = 2
    _BACK_LEFT_ID = 3
    def __init__(self):
        self.ir_sensor = [0,0,0,0]
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
    
def userInputCallback(msg, right):
    if((not irState.get(irState._FRONT_RIGHT_ID) or not irState.get(irState._FRONT_LEFT_ID)) and msg.data > 0 and USE_EMERGENCYSTOP): # check if it about to collide in the front
        print ("ABOUT TO COLLIDE FRONT EMERGENCY BRAKE")
        msg.data = 0
    elif ((not irState.get(irState._BACK_RIGHT_ID) or not irState.get(irState._BACK_LEFT_ID)) and msg.data < 0 and USE_EMERGENCYSTOP): # check if it about to collide in the back
        print ("ABOUT TO COLLIDE BACK EMERGENCY BRAKE")
        msg.data = 0

    if(right):
        pub_motor_r.publish(msg.data)
    else:
        pub_motor_l.publish(msg.data)
    return

if __name__ == "__main__":
    # init main loop
    rospy.init_node('Ir_drive_controller')
    
    # initialize wheels publisher
    pub_motor_l = rospy.Publisher(LEFT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    pub_motor_r = rospy.Publisher(RIGHT_MOTOR_TOPIC_OUTPUT, Int16, queue_size=1)
    
    # initialize subscriber for user input 
    user_input_sub_r = rospy.Subscriber(RIGHT_MOTOR_TOPIC_INPUT, Int16, userInputCallback, True)
    user_input_sub_l = rospy.Subscriber(LEFT_MOTOR_TOPIC_INPUT, Int16, userInputCallback, False)

    #Initialize subscriber for Ir sensor
    ir_sub = rospy.Subscriber(IR_TOPIC, Int16MultiArray, irSensorCallback)

    print("publishing to /roboy/pinky/middleware/espchair/wheels/assisted_navigation. Spinning...")
    rospy.spin()