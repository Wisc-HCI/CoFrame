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

        super(PrinterMachineNode,self).__init__(
            uuid, prefix, 
            init_fnt=self._init,
            start_fnt=self._start,
            stop_fnt=self._stop,
            pause_fnt=self._pause)

    def spin(self):
        pass

    def _init(self):
        pass

    def _start(self):
        pass

    def _stop(self):
        pass

    def _pause(self, state):
        pass

    def _call_to_register(self):
        pass