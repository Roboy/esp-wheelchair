class RepelentMode:
    """ Repelent field mode get slower as the robot get near an obstacle """
    DistFront = 1 # store nearest distance to front obstacle
    DistBack = 1 # store nearest distance to back obstacle
    Function = 1 # function defaults to linear
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
    
    def setFunction(self, function):    
        """ set memeber variable function """
        print("Changing Repelelent field to ", function)
        self.Function = function

    def control(self, inputLinear, inputAngular):
        """ give an output based of the distance to an obstacle """
        if(self.Function == 1): # Linear
            outputLinear, outputAngular = self.linear(inputLinear,inputAngular)
        elif(self.Function == 2): # Quadratic
            outputLinear, outputAngular = self.quadratic(inputLinear,inputAngular)
        else: # if undefined defaults to linear
            outputLinear, outputAngular = self.quadratic(inputLinear,inputAngular)
        return outputLinear,outputAngular
    
    def linear(self, inputLinear,inputAngular):
        """ adjust speed linearly to the distance of nearest obstacle """
        if inputLinear > 0:
            outputLinear = self.DistFront
        elif inputLinear < 0:
            outputLinear = -1*self.DistBack
        else:
            outputLinear = 0
        return outputLinear, inputAngular
    
    def quadratic(self, inputLinear, inputAngular):
        """ adjust speed linearly to the distance of nearest obstacle """
        if inputLinear > 0:
            outputLinear = pow(self.DistFront,2)
        elif inputLinear < 0:
            outputLinear = -1*pow(self.DistFront,2)
        else:
            outputLinear = 0
        return outputLinear, inputAngular