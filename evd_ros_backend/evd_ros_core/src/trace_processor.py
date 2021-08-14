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
from evd_sim.pose_reached import poseReached
from evd_sim.pose_interpolator import PoseInterpolator
from evd_sim.joints_stabilized import JointsStabilizedFilter
from evd_interfaces.frontend_interface import FrontendInterface
from evd_sim.joint_interpolator import JointInterpolator


SPIN_RATE = 5
UPDATE_RATE = 1000
JSF_NUM_STEPS = 20
JSF_DISTANCE_THRESHOLD = 0.001
POSITION_DISTANCE_THRESHOLD = 0.05
ORIENTATION_DISTANCE_THRESHOLD = 0.02


class TraceProcessor:

    def __init__(self, config_path, config_file_name):
        self._path = None
        self._trace_data = None

        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)

        self._fixed_frame = self._config['fixed_frame']
        self._link_groups = self._config['link_groups']
        self._joint_names = self._config['joint_names']

        frontend = FrontendInterface(use_processor_configure=True)
        self._job_queue = JobQueue('trace', self._start_job, self._end_job, frontend=frontend)
        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet'), self._config['pybullet'], gui=True)
        self.jsf = JointsStabilizedFilter(JSF_NUM_STEPS, JSF_DISTANCE_THRESHOLD)

        self._timer = rospy.Timer(rospy.Duration(1/UPDATE_RATE), self._update_cb)

    def _start_job(self, data):
        dct = json.loads(data)
        trajectory = NodeParser(dct['trajectory'])
        points = [NodeParser(p) for p in dct['points']]
        self._trace_data = {
            #TODO fill this in for update
        }
        #TODO produce path from trajectory and points
        self._path = []

        self.jsf.clear()
        self.ltk.set_joints(jp) # or jog to pose?
        self.pyb.set_joints(jp, jn)

    def _end_job(self, status, submit_fnt):
        self._trajectory = None
        self._points = None

        #TODO pack trace
        trace = Trace()
        self._trace_data = None

        data = trace.to_dct() if status else None
        submit_fnt(json.dumps(data))

    def _update_cb(self):
        # If the job has been started
        # step through trajectory and record the joints / frames as they occur
        if self._path != None and self._trace_data != None:
            pass

    def spin(self):
        rate = rospy.Rate(SPIN_RATE)
        while not rospy.is_shutdown():
            self._job_queue.update()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('trace_processor')

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")

    node = TraceProcessor(config_path, config_file_name)
    node.spin()