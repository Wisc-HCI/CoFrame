'''
???
'''


#TODO publish at_start, at_end, and lockout

class ProgramRunner:

    def __init__(self, raw_program):
        self._program = raw_program

        # "reserved-machine-cnc"

        #TODO

    def start(self):
        # bring robot to initial state
        self.reset()

        #TODO

    def stop(self):
        pass #TODO

    def pause(self):
        pass #TODO

    def unpause(self):
        pass #TODO

    def update(self):
        pass #TODO

    def reset(self):
        pass #TODO

    def step_forward(self):
        pass #TODO

    def step_backward(self):
        pass #TODO


class ProgramHooks:
    pass
