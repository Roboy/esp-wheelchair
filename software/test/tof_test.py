import unittest
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import pcl
import ros_numpy
import numpy as np
import pcl.pcl_visualization
from manual import * 


class Assisted_Navigation_Test(unittest.TestCase):
    def test_Repelent_Field(self):
        repelentMode = RepelentMode()
        repelentMode.control()
    def test_Manual_Mode(self):

if __name__ == '__main__':
    unittest.main()
