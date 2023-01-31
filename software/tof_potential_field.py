# # Usage:

# # cd esp-wheelchair
# # python3 software/tof_potential_field.py

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import pcl
import pcl_conversions

outputspeed = 0.0
outputangular = 0.0
inputspeed = 0.0
inputAngular = 0.0
emergencyStopThreshold = 0.1

def point_cloud_callback(msg):
    # Convert the PointCloud2 message to a PCL point cloud
    cloud = pcl.PointCloud_PointXYZ()
    pcl_conversions.from_ros_message(msg, cloud)

    # Process the point cloud here
    num_points = len(cloud.points)
    minDist = 99999
    for point in cloud.points:
        z = point[2]
        if minDist > z:
            minDist = z

    if minDist > 1: #thresholding maximum speed
        minDist = 1
    if minDist < 0: #thresholding minimum speed
        minDist = 0

    rospy.loginfo("minDist: %f", minDist)

    if minDist < emergencyStopThreshold: # emergency stop
        speed = 0.0

def user_input_callback(msg):
    # global userspeed, angular
    inputspeed = msg.linear.x
    inputAngular = msg.angular.z

def main():
    rospy.init_node('tof_potential_field')

    potential_field_pub = rospy.Publisher('/roboy/pinky/middleware/espchair/wheels/potential_field', Twist, queue_size=10)
    user_input_sub = rospy.Subscriber('/cmd_vel', Twist, user_input_callback)
    point_cloud_sub = rospy.Subscriber('/royale_camera_driver/point_cloud', PointCloud2, point_cloud_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = outputspeed
        twist.angular.z = inputAngular
        potential_field_pub.publish(twist)

        rospy.loginfo("publishing to /roboy/pinky/middleware/espchair/wheels/potential_field. Spinning...")
        rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    main()