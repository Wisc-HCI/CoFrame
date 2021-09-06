#!/usr/bin/env python3

'''
Presents a virtual assembly jig machin to EvD.
'''

import rospy

from evd_script import Machine, CubeRegion, Position, Orientation, ThingType, \
    CollisionMesh, Placeholder, Thing

from evd_interfaces.machine_template import MachineTemplate
from evd_interfaces.frontend_interface import FrontendInterface


PROCESS_TIME = 0 # This machine is just to serve as a passive real-world device


class AssemblyJigMachine(MachineTemplate):

    def __init__(self, prefix=None, rate=5, simulated=True):
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