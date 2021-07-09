import math

class AUVController(object):
    def __init__(self, state):
        self.__auv_state = state
        self.__turn_direction = "none"
        self.__heading = 0.0

    def get_desired_heading(self):
        return self.__heading #returns the new heading we need to set

    def decide(self, state, gnext, rnext):
        #figures out the command needed
        #returns either None for no command, or valid command

        self.__auv_state = state

        #calculate and return the necessary heading

        #strategy 1: find the midpoint and turn there

        position = self.__auv_state['position'] #get the current position of the AUV

        x_a = position[0]
        y_a = position[1]

        x_g = gnext[0]
        y_g = gnext[1]

        x_r = rnext[0]
        y_r = rnext[1]

        x_mid = (x_g + x_r) / 2.0
        y_mid = (y_g + y_r) / 2.0
        
        target_heading = math.atan2(x_mid-x_a,y_mid-y_a) #rotation necessary to reach the midpoint of the two buoys

        target_heading = math.degrees(target_heading) #convert to degrees

        #this only returns the target heading, not the angle offset

        self.__heading = target_heading

        #figure out which direction to turn
        if target_heading > self.__heading:
            self.__turn_direction = "right"
        elif target_heading < self.__heading:
            self.__turn_direction = "left"
        else:
            self.__turn_direction = "none"

        #print("Offset: " + str(angle_offset))

        #return the command
        command = ""

        if self.__turn_direction == "right":
            command = "RIGHT STANDARD RUDDER"
        elif self.__turn_direction == "left":
            command = "LEFT STANDARD RUDDER"
        else:
            command = "RUDDER AMIDSHIPS"

        return command

#turn notes
#update to the state every 1 second
#can change the gates to make them farther apart
#need to add or subtract 360 for the angle