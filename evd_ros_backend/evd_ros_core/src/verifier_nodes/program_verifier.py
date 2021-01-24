#!/usr/bin/env python

import json
import rospy

from evd_interfaces.data_client_interface import DataClientInterface
from evd_interfaces.symbolic_program_runner import SymbolicProgramRunner


class Verifier:

    def __init__(self):
        self._trace_table = {}
        self._pose_table = {}

        self._data_client = DataClientInterface(on_program_update_cb=self._verify_program)

    def _verify_program(self):

        # Run program symbolically to verify program logic (ie things are moved, machines are satisfied)
        pass

        # Handle grasp / release feasability
        pass

    def spin(self):
        pass


if __name__ == "__main__":
    rospy.init_node('verifier')

    node = Verifier()
    node.spin()
