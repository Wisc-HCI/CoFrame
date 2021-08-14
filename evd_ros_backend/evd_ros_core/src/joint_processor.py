#!/usr/bin/env python3

'''
Confirms reachability of waypoint or location. Generates a joint datastructure.
'''

import os
import json
import rospy

from evd_script import Joints, NodeParser
from evd_sim.pose_reached import poseReached
from evd_interfaces.job_queue import JobQueue
from evd_sim.lively_tk_solver import LivelyTKSolver
from evd_sim.joints_stabilized import JointsStabilizedFilter


SPIN_RATE = 5
UPDATE_RATE = 1000
JSF_NUM_STEPS = 20
JSF_DISTANCE_THRESHOLD = 0.001
POSITION_DISTANCE_THRESHOLD = 0.05
ORIENTATION_DISTANCE_THRESHOLD = 0.02


class JointProcessor:
    
    def __init__(self, config_path, config_file_name):
        self._target = None
        self._joints = None

        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)
        
        self._fixed_frame = self._config['fixed_frame']
        self._ee_frame = self._config['link_groups']['end_effector_path']
        self._joint_names = self._config['joint_names']
        
        self._job_queue = JobQueue('joints', self._start_job, self._end_job)
        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.jsf = JointsStabilizedFilter(JSF_NUM_STEPS, JSF_DISTANCE_THRESHOLD)

        self._timer = rospy.Timer(rospy.Duration(1/UPDATE_RATE), self._update_cb)

    def _start_job(self, data):
        length = len(self._joint_names)
        waypoint = NodeParser(data)
        self._target = waypoint.to_ros()
        self._joints = Joints(
            length=length,
            joint_names=self._joint_names,
            joint_positions=[0]*length,
            reachable=False)

        self.jsf.clear()
        self.ltk.reset()

    def _end_job(self, status, submit_fnt):
        self._target = None
        data = self._joints.to_dct() if status else None
        self._joints = None
        submit_fnt(json.dumps(data))

    def _update_cb(self):
        # If the job has been started
        # step toward target and record the joint positions
        # and when joints are stable (little/no-more optimization) then
        # check whether the target pose was reached within a margin of error
        if self._target != None and self._joints != None:

            (jp, jn), frames = self.ltk.step(self._target)
            ee_pose = LivelyTKSolver.get_ee_pose(frames[0])
            self.jsf.append(jp)

            self._joints.set_joint_positions_by_names(jp,jn)

            if self.jsf.isStable():
                self._joints.reachability = poseReached(self._target, ee_pose, 
                                                        POSITION_DISTANCE_THRESHOLD, 
                                                        ORIENTATION_DISTANCE_THRESHOLD)
                self._job_queue.completed()

    def spin(self):
        rate = rospy.Rate(SPIN_RATE)
        while not rospy.is_shutdown():
            self._job_queue.update()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('grasp_verifier')

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")

    node = JointProcessor(config_path, config_file_name)
    node.spin()