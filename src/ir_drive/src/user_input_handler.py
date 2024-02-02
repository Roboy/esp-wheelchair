from std_msgs.msg import Float64, Int16

class UserInputHandler:
    """ Handles PWM input to linear and angular speed  """
    PWM_MIN = 0 # PWM minimum value
    PWMRANGE = 0 # PWM range value
    
    leftInput = 0
    leftIsDefined = False
    rightInput = 0
    rightIsDefined = False
    def __init__(self, PWM_MIN, PWMRANGE):
        self.PWM_MIN = PWM_MIN
        self.PWMRANGE = PWMRANGE
        return
    
    def setUserInput(self, value, right):
        """ Set the user input, store them at class member and define IsDefine as true """
        if (right):
            self.rightInput = value
            self.rightIsDefined = True
        else : 
            self.leftInput = value
            self.leftIsDefined = True
        return
    
    def getUserInput(self):
        """ Make sure User input is defined and return user input as linear and angular speed returned the user input in the range of -1 to 1"""
        if(self.leftIsDefined and self.rightIsDefined):
            inputLinear = self.translate((self.rightInput.data + self.leftInput.data) / 2, -self.PWMRANGE, self.PWMRANGE, -1, 1) 
            inputAngular = self.translate((self.rightInput.data - self.leftInput.data) / 2, -self.PWMRANGE, self.PWMRANGE, -1, 1)
            return inputLinear, inputAngular
    
        else:
            inputLinear = 0
            inputAngular = 0
            return inputLinear, inputAngular

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # calculate value range
        leftRange = leftMax - leftMin
        rightRange = rightMax - rightMin

        # Convert the left range into a 0-1 range
        valueScaled = float(value - leftMin) / float(leftRange)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightRange)
