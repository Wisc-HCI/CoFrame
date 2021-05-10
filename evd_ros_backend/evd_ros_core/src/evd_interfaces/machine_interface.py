
import rospy


class MachineInterface:

    def __init__(self, prefix):
        self._prefix = prefix

        self.start_pub
        self.stop_pub
        self.pause_pub

        self.ack_sub
        self.wait_sub


    def estop(self, machineUuid):
        pass

    def pause(self, state, machineUuid):
        pass

    def initialize(self, machineUuid):
        pass

    def start(self, machineUuid):
        pass

    def stop(self, machineUuid):
        pass

    def get_status(self, machineUuid):
        return 'running' #TODO
