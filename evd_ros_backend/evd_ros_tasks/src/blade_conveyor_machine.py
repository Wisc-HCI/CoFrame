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


class BladeConveyorMachine(MachineTemplate):

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