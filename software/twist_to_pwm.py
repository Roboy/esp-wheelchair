# Usage:


# rosrun rosserial_python serial_node.py tcp
# plugin wheelchair power

# cd esp-wheelchair
# python software/twist_to_pwm.py

# start ROS joystick node - atk3 is for logitech joystick
# roslaunch teleop_twist_joy teleop.launch joy_config:=atk3
# hold 1 and left joystick to drive (hold 2 to drive faster)

# OR

# alternatively run keyboard control
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# read the commands for the keyboard!



import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

PWM_MIN = 5
PWMRANGE = 40

topic_root = "/roboy/pinky"
rospy.init_node("wheelchair_twist_converter")

pub_l = rospy.Publisher(topic_root + "/middleware/espchair/wheels/left", Int16, queue_size=1)
pub_r = rospy.Publisher(topic_root + "/middleware/espchair/wheels/right", Int16, queue_size=1)

sign = lambda a: (a>0) - (a<0)

def mapPwm(x, out_min, out_max):
	return x * (out_max - out_min) + out_min;


def cb(msg):
	if not rospy.get_param('wheelchair_emergency_stopped'):
		rospy.loginfo_throttle(5, "Publishing pwm..")
		x = max(min(msg.linear.x, 1.0), -1.0)
		z = max(min(msg.angular.z, 1.0), -1.0)

		l = (msg.linear.x - msg.angular.z) / 2.0
		r = (msg.linear.x + msg.angular.z) / 2.0

		lPwm = mapPwm(abs(l), PWM_MIN, PWMRANGE)
		rPwm = mapPwm(abs(r), PWM_MIN, PWMRANGE)

		pub_l.publish(sign(l)*lPwm)
		pub_r.publish(sign(r)*rPwm)
	else:
		rospy.logwarn_throttle(1, "Emergency stop active. Ignoring cmd_vel")



sub = rospy.Subscriber(topic_root + "/control/espchair/cmd_vel", Twist, cb)

rospy.loginfo("Subscribed to " + topic_root + "/control/espchair/cmd_vel" + ", will publish wheelchair PWM. Spinning...")
rospy.spin()
