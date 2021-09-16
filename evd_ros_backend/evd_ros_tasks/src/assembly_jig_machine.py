#!/usr/bin/env python3

'''
Presents a virtual assembly jig machin to EvD.
'''

import rospy

from evd_interfaces.machine_template import MachineTemplate


PROCESS_TIME_ASSEMBLY_JIG = 0
MACHINE_UUID_ASSEMBLY_JIG = 'assembly-jig-machine-uuid'


class AssemblyJigMachine(MachineTemplate):

    def __init__(self, uuid, prefix=None, rate=5, simulated=True):
        self._paused = False
        self._rate = rate
        self._simulated = simulated

        self._times = []
        self._currentTime = 0

        super(AssemblyJigMachine,self).__init__(
            uuid, prefix, 
            init_fnt=self._init,
            start_fnt=self._start,
            stop_fnt=self._stop,
            pause_fnt=self._pause)

    def spin(self):
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():

            if self.current_status == 'running' and self.elapsedTime() >= PROCESS_TIME_ASSEMBLY_JIG:
                self.current_status = 'idle'
            else:
                self.update_status()

            rate.sleep()

    def _init(self):
        if not self.is_running:
            self.current_status = 'idle'
            return True
        else:
            self._stop(True)
            return False

    def _start(self):
        if self.current_status == 'idle':
            self.current_status = 'running'
            self._paused = False

            self._times = [0] 
            self._currentTime = time.time()

            return True
        else:
            return False

    def _stop(self):
        if emergency:
            self.current_status = 'error'
        elif self.elapsedTime < PROCESS_TIME_ASSEMBLY_JIG: # if we stop early
            self.current_status = 'error'
        else:
            self.current_status = 'idle'
        return True

    def _pause(self, state):
        if self.is_running:
            self._paused = state

            if self._paused:
                self._times.append(time.time() - self._currentTime) # save time cause we are pausing
                self.current_status = 'paused'
            else:
                self._currentTime = time.time() #unpausing so start checking time again
                self.current_status = 'running'
            
            return True
        else:
            return False

    @property
    def elapsedTime(self):
        return sum(self._times) + (time.time() - self._currentTime)


if __name__ == "__main__":
    rospy.init_node('assembly_jig_machine')

    uuid = rospy.get_param('uuid',MACHINE_UUID_ASSEMBLY_JIG)
    prefix = rospy.get_param('prefix',None)
    rate = rospy.get_param('rate',5)
    simulated = rospy.get_param('simulated',True)

    node = AssemblyJigMachine(uuid,prefix,rate,simulated)
    node.spin()