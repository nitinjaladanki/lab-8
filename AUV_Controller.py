import math

class AUVController(object):
    def __init__(self, state):
        self.__auv_state = state
        self.__turn_direction = "none"
        self.__heading = None

    def get_desired_heading(self):
        return self.__heading #returns the new heading we need to set

    def decide(self, state, gnext, rnext):
        #figures out the command needed
        #returns either None for no command, or valid command

        self.__auv_state = state

        #calculate and return the necessary heading

        #strategy 1: find the midpoint and turn there

        position = self.__auv_state['position'] #get the current position of the AUV

        x_a,y_a = position

        x_g,y_g = gnext

        x_r,y_r = rnext

        x_mid = (x_g + x_r) / 2.0
        y_mid = (y_g + y_r) / 2.0
        
        angle_offset = math.atan2(x_a-x_mid,y_a-y_mid) #rotation necessary to reach the midpoint of the two buoys

        angle_offset = math.degrees(angle_offset) #convert to degrees

        self.__heading = self.__auv_state['heading'] + angle_offset

        #figure out which direction to turn

        if angle_offset < 90: #negative turn is to the right
            self.__turn_direction = "right"
        elif angle_offset > 90: #positive turn is to the left
            self.__turn_direction = "left"
        else: #no turn necessary
            self.__turn_direction = "none"

        #return the command
        command = ""

        if self.__turn_direction == "right":
            command = "RIGHT STANDARD RUDDER"
        elif self.__turn_direction == "left":
            command = "LEFT STANDARD RUDDER"
        else:
            command = "RUDDER AMIDSHIPS"

        return command