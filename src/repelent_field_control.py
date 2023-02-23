class RepelentMode:
    """ Repelent field mode get slower as the robot get near an obstacle """
    DistFront = 1
    DistBack = 1
    def __init__(self):
        return
    
    def setDistanceFront(self, distance):
        """ Set member variable DistFront """
        self.DistFront = distance
    def getDistanceFront(self):
        """ Set member variable DistFront """
        return self.DistFront 
    
    def setDistanceBack(self, distance):
        """ Set member variable DistBack """
        self.DistBack = distance
    def getDistanceBack(self):
        """ Set member variable DistFront """
        return self.DistBack 
    def control(self, inputLinear, inputAngular):
        """ give an output based of the distance to an obstacle """
        if inputLinear >= 0:
            outputLinear = self.DistFront
        elif inputLinear < 0:
            outputLinear = -1*self.DistBack

        return outputLinear,inputAngular