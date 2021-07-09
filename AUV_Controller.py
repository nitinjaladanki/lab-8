class AUV_Controller(object):
    def __init__(self, state):
        self.__auv_state = state

    def get_desired_heading(self):
        #calculate and return the necessary heading
        return None

    def decide(self, state, gnext, bnext):
        #figure out the command needed
        self.__auv_state = state
        
        return None