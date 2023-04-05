class IRState:
    """ Manual Mode no modification between user input and output """
    _FRONT_RIGHT = 1
    _FRONT_LEFT = 2
    _BACK_RIGHT = 3
    _BACK_LEFT = 4
    def __init__(self):
        self.dist_front_right = 0
        self.dist_front_left = 0
        self.dist_back_right = 0
        self.dist_back_left = 0
        return
    
    def set(self, input, pos):
        if pos == IRState._FRONT_RIGHT:
            self.dist_front_right = input
        elif pos == IRState._FRONT_LEFT:
            self.dist_front_left = input
        elif pos == IRState._BACK_RIGHT:
            self.dist_back_right = input
        elif pos == IRState._BACK_LEFT:
            self.dist_back_left = input

    def get(self,pos):
        if pos == IRState._FRONT_RIGHT:
            return self.dist_front_right
        elif pos == IRState._FRONT_LEFT:
            return self.dist_front_left
        elif pos == IRState._BACK_RIGHT:
            return self.dist_back_right
        elif pos == IRState._BACK_LEFT:
            return self.dist_back_left



    def control(self, inputLinear, inputAngular):
        return inputLinear,inputAngular