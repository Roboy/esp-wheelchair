#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

import Jetson.GPIO as GPIO

print("Initializing IR Publisher")
# Set up GPIO pins
pin_front_right = 7
pin_front_left = 11
pin_back_right = 13
pin_back_left = 15

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin_front_right, GPIO.IN)
GPIO.setup(pin_front_left, GPIO.IN)
GPIO.setup(pin_back_right, GPIO.IN)
GPIO.setup(pin_back_left, GPIO.IN)

# Set up ROS node
rospy.init_node("Ir_publisher")
print("Initializing IR_publisher initialized")

# Set up publishers for each IR sensor
DIST_FRONT_RIGHT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/front_right"
DIST_FRONT_LEFT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/front_left"
DIST_BACK_RIGHT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/back_right"
DIST_BACK_LEFT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/back_left"

pub_front_right = rospy.Publisher(DIST_FRONT_RIGHT_TOPIC, Int16, queue_size=10)
pub_front_left = rospy.Publisher(DIST_FRONT_LEFT_TOPIC, Int16, queue_size=10)
pub_back_right = rospy.Publisher(DIST_BACK_RIGHT_TOPIC, Int16, queue_size=10)
pub_back_left = rospy.Publisher(DIST_BACK_LEFT_TOPIC, Int16, queue_size=10)

def main():
    while True:
        # read value from  sensor and publish publish them to their topic
        # front right
        value_front_right = GPIO.input(pin_front_right)
        print("front right : " + str(value_front_right))
        pub_front_right.publish(value_front_right)

        # front left 
        value_front_left = GPIO.input(pin_front_left)
        print("front left : " + str(value_front_left))
        pub_front_left.publish(value_front_left)

        # back right
        value_back_right = GPIO.input(pin_back_right)
        print("back right : " + str(value_back_right))
        pub_back_right.publish(value_back_right)
        
        # back left
        value_back_left = GPIO.input(pin_back_left)
        print("back left : " + str(value_back_left))
        pub_back_left.publish(value_back_left)


main()