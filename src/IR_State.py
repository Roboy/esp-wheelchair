class IRState:
    """ Manual Mode no modification between user input and output """
    _FRONT_RIGHT_ID = 1
    _FRONT_LEFT_ID = 2
    _BACK_RIGHT_ID = 3
    _BACK_LEFT_ID = 4
    def __init__(self):
        self.ir_sensor = [0,0,0,0]
        return
    
    def set(self, input):
        self.ir_sensor = input
        return 

    def get(self,pos):
        return self.ir_sensor[pos]