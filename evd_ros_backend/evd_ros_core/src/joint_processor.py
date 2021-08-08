#!/usr/bin/env python3

'''
Confirms reachability of waypoint or location. Generates a joint datastructure.
'''

import os
import json
import rospy

from evd_script import Joints
from evd_interfaces.job_queue import JobQueue
from evd_sim.lively_tk_solver import LivelyTKSolver
from evd_sim.pybullet_model import PyBulletModel


class JointProcessor:
    
    def __init__(self, config_path, config_file_name):
        self._joints = None

        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)
        
        self._fixed_frame = self._config['fixed_frame']
        self._ee_frame = self._config['link_groups']['end_effector_path']
        self._joint_names = self._config['joint_names']
        
        self._job_queue = JobQueue('joints', self._start_job, self._end_job)
        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet/'), self._config['pybullet'], gui=True)

    def _start_job(self, data):
        
        self._joints = Joints()
        #TODO write this to hook into lively-tk
        self.ltk.step()
        self.pyb.step()


    def _end_job(self, status, submit_fnt):
        data = self._joints.to_dct() if status else None
        submit_fnt(json.dumps(data))

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._job_queue.update()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('grasp_verifier')

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")

    node = JointProcessor(config_path, config_file_name)
    node.spin()