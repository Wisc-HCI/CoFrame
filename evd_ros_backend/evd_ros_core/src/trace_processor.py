#!/usr/bin/env python3

'''
Converts abstract trajectories into executed robot movement traces.

Also applies graders to the traces to produce grade appraisals
'''

import os
import json
import rospy

from evd_script import Trace, NodeParser
from evd_interfaces.job_queue import JobQueue
from evd_sim.lively_tk_solver import LivelyTKSolver
from evd_sim.pybullet_model import PyBulletModel


class TraceProcessor:

    def __init__(self, config_path, config_file_name):
        self._trace = None

        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)

        self._fixed_frame = self._config['fixed_frame']
        self._link_groups = self._config['link_groups']
        self._joint_names = self._config['joint_names']

        self._job_queue = JobQueue('trace', self._start_job, self._end_job)
        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet/'), self._config['pybullet'], gui=True)

    def _start_job(self, data):
        dct = json.loads(data)
        trajectory = NodeParser(dct['trajectory'])
        points = [NodeParser(p) for p in dct['points']]

        self._trace = Trace()

        self.ltk.step()
        self.pyb.step()
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

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")

    node = TraceProcessor(config_path, config_file_name)
    node.spin()