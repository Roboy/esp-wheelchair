import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

PWM_MIN = 10
PWMRANGE = 160

rospy.init_node("wheelchair_twist_converter")

pub_l = rospy.Publisher("/espchair/wheel_L", Int16, queue_size=1)
pub_r = rospy.Publisher("/espchair/wheel_R", Int16, queue_size=1)

sign = lambda a: (a>0) - (a<0)

def mapPwm(x, out_min, out_max):
	return x * (out_max - out_min) + out_min;


def cb(msg):
	x = max(min(msg.linear.x, 1.0), -1.0)
	z = max(min(msg.angular.z, 1.0), -1.0)

	l = (msg.linear.x - msg.angular.z) / 2.0
	r = (msg.linear.x + msg.angular.z) / 2.0

	lPwm = mapPwm(abs(l), PWM_MIN, PWMRANGE)
	rPwm = mapPwm(abs(r), PWM_MIN, PWMRANGE)

	pub_l.publish(sign(l)*lPwm)
	pub_r.publish(sign(r)*rPwm)



sub = rospy.Subscriber("/cmd_vel", Twist, cb)


rospy.spin()