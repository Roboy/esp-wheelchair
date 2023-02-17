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
sys.path.insert(1, '../esp-wheelchair/software')

from manual import * 
from repelent_field import * 

class Assisted_Navigation_Test(unittest.TestCase):
    def test_Repelent_Field(self):
        repelentMode = RepelentMode()
        inputLinear = 10
        inputAngular = 11
        minDistFront = 0.01
        minDistBack = 10
        
        outputLiner,outputAngular = repelentMode.control(inputLinear,inputAngular, minDistFront, minDistBack)
        # test
        # assert(outputLiner == 0 and outputAngular == outputAngular )
        self.assertEqual((outputLiner, outputAngular), (0.01, inputAngular))
        repelentMode = RepelentMode()
        inputLinear = -1
        inputAngular = 2
        minDistFront = 2
        minDistBack = 0.1
        
        outputLiner,outputAngular = repelentMode.control(inputLinear,inputAngular, minDistFront, minDistBack)
        # test
        self.assertEqual((outputLiner, outputAngular), (-0.1, inputAngular))


    def test_Manual_Mode(self):
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
