#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

import Jetson.GPIO as GPIO

# Set up GPIO pins
pin_front_right = 1
pin_front_left = 2
pin_back_right = 3
pin_back_left = 4

GPIO.setmode(GPIO.BOARD)
GPIO.setup(pin_front_right, GPIO.IN)
GPIO.setup(pin_front_left, GPIO.IN)
GPIO.setup(pin_back_right, GPIO.IN)
GPIO.setup(pin_back_left, GPIO.IN)

# Set up ROS node
rospy.init_node("Ir_handler")

# Set up publishers for each distance topic
DIST_FRONT_RIGHT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/front_right"
DIST_FRONT_LEFT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/front_left"
DIST_BACK_RIGHT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/back_right"
DIST_BACK_LEFT_TOPIC = "/roboy/pinky/middleware/espchair/wheels/dist/back_left"

pub_front_right = rospy.Publisher(DIST_FRONT_RIGHT_TOPIC, Int16, queue_size=10)
pub_front_left = rospy.Publisher(DIST_FRONT_LEFT_TOPIC, Int16, queue_size=10)
pub_back_right = rospy.Publisher(DIST_BACK_RIGHT_TOPIC, Int16, queue_size=10)
pub_back_left = rospy.Publisher(DIST_BACK_LEFT_TOPIC, Int16, queue_size=10)

# Define callback function for GPIO pin changes
def gpio_callback(channel):
    if channel == pin_front_right:
        pub_front_right.publish(GPIO.input(channel))
    elif channel == pin_front_left:
        pub_front_left.publish(GPIO.input(channel))
    elif channel == pin_back_right:
        pub_back_right.publish(GPIO.input(channel))
    elif channel == pin_back_left:
        pub_back_left.publish(GPIO.input(channel))

# Set up GPIO pin event detection
GPIO.add_event_detect(pin_front_right, GPIO.BOTH, callback=gpio_callback)
GPIO.add_event_detect(pin_front_left, GPIO.BOTH, callback=gpio_callback)
GPIO.add_event_detect(pin_back_right, GPIO.BOTH, callback=gpio_callback)
GPIO.add_event_detect(pin_back_left, GPIO.BOTH, callback=gpio_callback)

# Spin ROS node
rospy.spin()

# Clean up GPIO pins
GPIO.cleanup()
