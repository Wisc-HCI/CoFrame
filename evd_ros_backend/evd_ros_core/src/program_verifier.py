#!/usr/bin/env python3

'''
Runs a full check on program and attempts to repair orphans
'''

import json
import rospy

from evd_interfaces.job_queue import JobQueue
from evd_interfaces.frontend_interface import FrontendInterface


class Verifier:

    def __init__(self):
        self._trace_table = {}
        self._pose_table = {}

        self._frontend = FrontendInterface(use_update=True, update_cb=self._program_update_cb)
        self._job_queue = JobQueue('program_verification', self._start_job, self._end_job, self._frontend)

    def _program_update_cb(self, program):
        pass

    def _start_job(self, data):
        # Run program symbolically to verify program logic (ie things are moved, machines are satisfied)
        pass

    def _end_job(self, status, submit_fnt):
        pass

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._job_queue.update()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('verifier')

    node = Verifier()
    node.spin()
