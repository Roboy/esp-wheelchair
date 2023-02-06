class RepelentMode:
    def __init__(self):
        return
    
    def control(self, inputLinear, inputAngular, minDistFront, minDistBack):
        if inputLinear > 0:
            outputLinear = minDistFront
        elif inputLinear < 0:
            outputLinear = minDistBack

        return outputLinear,inputAngular