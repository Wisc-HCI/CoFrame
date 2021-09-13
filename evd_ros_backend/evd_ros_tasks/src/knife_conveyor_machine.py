#!/usr/bin/env python3

'''
Presents a virtual assembly jig machin to EvD.
'''

import rospy

from evd_interfaces.machine_template import MachineTemplate


MACHINE_UUID_KNIFE_CONVEYOR = 'knife-conveyor-machine-uuid'
PROCESS_TIME_KNIFE_CONVEYOR = 5


class KnifeConveyorMachine(MachineTemplate):

    def __init__(self):
        pass

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