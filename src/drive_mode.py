class DriveMode:
    """ this class is a place holder for the current active class """
    def __init__(self,mode):
        self.Mode = mode
        return
    
    def setMode(self, mode):
        self.Mode = mode
        return

    def control(self, inputLinear, inputAngular):
        return self.Mode.control(inputLinear,inputAngular)