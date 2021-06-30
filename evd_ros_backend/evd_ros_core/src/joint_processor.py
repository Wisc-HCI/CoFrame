#!/usr/bin/env python3

'''
Confirms reachability of waypoint or location. Generates a joint datastructure.
'''

import json
import rospy

from evd_script import Joints
from evd_interfaces.job_queue import JobQueue


class JointProcessor:
    
    def __init__(self, config_file):
        self._joints = None

        with open(config_file,'r') as f:
            config = json.load(f)
            self._fixed_frame = config['fixed_frame']
            self._ee_frame = config['link_groups']['end_effector_path']
            self._joint_names = config['joint_names']
        
        self._job_queue = JobQueue('joints', self._start_job, self._end_job)

    def _start_job(self, data):
        
        self._joints = Joints()
        #TODO write this to hook into lively-tk

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

    config_file = rospy.get_param('~config_file')

    node = JointProcessor(config_file)
    node.spin()