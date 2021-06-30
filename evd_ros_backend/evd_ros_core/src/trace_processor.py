#!/usr/bin/env python3

'''
Converts abstract trajectories into executed robot movement traces.

Also applies graders to the traces to produce grade appraisals
'''

import json
import rospy

from evd_script import Trace, NodeParser
from evd_interfaces.job_queue import JobQueue


class TraceProcessor:

    def __init__(self, config_file):
        self._trace = None

        with open(config_file,'r') as f:
            config = json.load(f)
            self._fixed_frame = config['fixed_frame']
            self._link_groups = config['link_groups']
            self._joint_names = config['joint_names']

        self._job_queue = JobQueue('trace', self._start_job, self._end_job)

    def _start_job(self, data):

        dct = json.loads(data)
        trajectory = NodeParser(dct['trajectory'])
        points = [NodeParser(p) for p in dct['points']]

        self._trace = Trace()
        #TODO write this to hook into pybullet (not sure how that looks yet)

    def _end_job(self, status, submit_fnt):
        data = self._trace.to_dct() if status else None
        submit_fnt(json.dumps(data))

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._job_queue.update()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('trace_processor')

    config_file = rospy.get_param('~config_file')

    node = TraceProcessor(config_file)
    node.spin()