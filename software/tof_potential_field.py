# # Usage:

# # cd esp-wheelchair
# # python3 software/tof_potential_field.py

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import pcl
import ros_numpy
import numpy as np
# import pcl_conversions
# import pcl.pcl_conversions
import pcl.pcl_visualization

# outputLinear = 0.0
# outputAngular = 0.0
# inputLinear = 0.0
# inputAngular = 0.0
emergencyStopThreshold = 0.01
viewer = pcl.pcl_visualization.PCLVisualizering()
    
visualDone = False

def point_cloud_callback(msg):
    
    # chaneg from pointcloud2 to numpy
    pc = ros_numpy.numpify(msg)
    height = pc.shape[0]
    width = pc.shape[1]
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)

    p = pcl.PointCloud(np.array(np_points, dtype=np.float32))
    viewer.AddPointCloud(p, b'scene_cloud', 0)
    viewer.SpinOnce()
    viewer.RemovePointCloud( b'scene_cloud', 0)
    # find the nearest point
    global minDist, maxDist
    minDist = 99999
    maxDist = -99999
    # print("np_points length :", len(np_points) / width)
    
    for i in range(len(np_points)):
        if(minDist > np_points[i][2]):
            minDist = np_points[i][2]
        if(maxDist < np_points[i][2]):
            maxDist = np_points[i][2]
    # minDist *=10
    rospy.loginfo("max: %f min : %f", maxDist, minDist )

    # if minDist > 0.7: #thresholding maximum speed
    #     minDist = 0.7
    # if minDist < 0: #thresholding minimum speed
    #     minDist = 0
    # global outputLinear
    # rospy.loginfo("width: %f height : %f", width,height )
    # if inputLinear < 0:
    #     outputLinear = -1 * minDist
    # elif inputLinear > 0:
    #     outputLinear = minDist
    # else : 
    #     outputLinear = 0
    
    # outputspeed = minDist
    if minDist < emergencyStopThreshold: # emergency stop
        outputspeed = 0.0
    


def user_input_callback(msg):
    global inputLinear, inputAngular, outputAngular
    inputLinear = msg.linear.x
    inputAngular = msg.angular.z
    # print("inputLinear : ", inputLinear, "inputAngular  : ",  inputAngular)
    outputAngular = inputAngular
    outputLinear = inputLinear
    if(minDist < emergencyStopThreshold and inputLinear > 0): # asume that this is the front ToF
        outputLinear = 0
    
    elif (minDist < emergencyStopThreshold and inputLinear < 0): # asume that this is the back ToF
        outputLinear = 0

    twist = Twist()
    print("outputLinear : ", outputLinear, "outputAngular  : ",  outputAngular)
    twist.linear.x = outputLinear
    twist.angular.z = outputAngular
    potential_field_pub.publish(twist)


    

# def main():
rospy.init_node('tof_potential_field')

potential_field_pub = rospy.Publisher('/roboy/pinky/middleware/espchair/wheels/potential_field', Twist, queue_size=10)
user_input_sub = rospy.Subscriber('/cmd_vel', Twist, user_input_callback)
point_cloud_sub = rospy.Subscriber('/royale_camera_driver/point_cloud', PointCloud2, point_cloud_callback)


# sub = rospy.Subscriber("/roboy/pinky/middleware/espchair/wheels/potential_field", Twist, cb)

# rate = rospy.Rate(10)
# while not rospy.is_shutdown():
#     twist = Twist()
#     print("outputLinear : ", outputLinear, "outputAngular  : ",  outputAngular)
#     twist.linear.x = outputLinear
#     twist.angular.z = outputAngular
#     potential_field_pub.publish(twist)

#     rospy.loginfo("publishing to /roboy/pinky/middleware/espchair/wheels/potential_field. Spinning...")
#     # rospy.spin()
#     rate.sleep()




rospy.loginfo("publishing to /roboy/pinky/middleware/espchair/wheels/potential_field. Spinning...")
rospy.spin()
# rate.sleep()

# if __name__ == '__main__':
#     main()