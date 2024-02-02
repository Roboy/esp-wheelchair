import unittest
import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import pcl
import ros_numpy
import numpy as np
import pcl.pcl_visualization
import sys
sys.path.insert(1, '../esp-wheelchair/src')

from manual_control import * 
from repelent_field_control import * 

class Assisted_Navigation_Test(unittest.TestCase):
    """ Class to test use cases of assisted navigation functions """

    def test_Repelent_Field(self):
        """ Test the Repelent Field class """
        repelentMode = RepelentMode()
        inputLinear = 10
        inputAngular = 11
        minDistFront = 0.01
        minDistBack = 10
        repelentMode.setDistanceFront(minDistFront)
        repelentMode.setDistanceBack(minDistBack)
        outputLiner,outputAngular = repelentMode.control(inputLinear,inputAngular)
        # test speed when going forward
        self.assertEqual((outputLiner, outputAngular), (0.01, inputAngular))
        repelentMode = RepelentMode()
        inputLinear = -1
        inputAngular = 2
        minDistFront = 2
        minDistBack = 0.1
        repelentMode.setDistanceBack(minDistFront)
        repelentMode.setDistanceBack(minDistBack)
        outputLiner,outputAngular = repelentMode.control(inputLinear,inputAngular)
        # test speed when going backward
        self.assertEqual((outputLiner, outputAngular), (-0.1, inputAngular))


    def test_Manual_Mode(self):
        """ Test Manual Mode should just return the same value """
        manualMode = ManualMode()
        inputLinear = 10
        inputAngular = 1
        self.assertEqual(manualMode.control(inputLinear,inputAngular), (inputLinear, inputAngular))

        inputLinear = -10
        inputAngular = 1
        self.assertEqual(manualMode.control(inputLinear,inputAngular), (inputLinear, inputAngular))

        inputLinear = 0
        inputAngular = -1
        self.assertEqual(manualMode.control(inputLinear,inputAngular), (inputLinear, inputAngular))

if __name__ == '__main__':
    unittest.main()
