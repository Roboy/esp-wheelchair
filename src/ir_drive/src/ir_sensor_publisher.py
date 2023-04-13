#!/usr/bin/python3
import Jetson.GPIO as GPIO
import time

import rospy
from std_msgs.msg import Int16MultiArray


SENSOR_PINS = [15,16,18,19,21,22,23,24]
TOPIC_NAME = "/roboy/pinky/sensing/distance"

def setup_gpios():
    GPIO.setmode(GPIO.BOARD)
    for pin in SENSOR_PINS:
        GPIO.setup(pin, GPIO.IN)

def publish_data(pub):
    msg = Int16MultiArray()
    for pin in SENSOR_PINS:
        msg.data.append(GPIO.input(pin))
    pub.publish(msg)

def main():
    setup_gpios()

    rospy.init_node("ir_sensor_publisher")
    pub = rospy.Publisher(TOPIC_NAME, Int16MultiArray, queue_size=1)
    rate = rospy.Rate(50)
    rospy.loginfo("Infrared distance node is setup. Publishing data on " + TOPIC_NAME)
    while not rospy.is_shutdown():
        publish_data(pub)
        rate.sleep()

if __name__ == '__main__':
   main()