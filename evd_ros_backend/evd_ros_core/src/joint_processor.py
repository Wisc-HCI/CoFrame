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
from evd_sim.pybullet_model import PyBulletModel
from evd_sim.joints_stabilized import JointsStabilizedFilter
from evd_interfaces.frontend_interface import FrontendInterface

TIMEOUT_COUNT = 1
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
        self._trace_data = None
        self._state = 'idle'
        self._updateCount = 0

        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)
        
        self._fixed_frame = self._config['fixed_frame']
        self._ee_frame = self._config['link_groups']['end_effector_path']
        self._joint_names = self._config['joint_names']
        
        frontend = FrontendInterface(use_processor_configure=True)
        self._job_queue = JobQueue('joints', self._start_job, self._end_job, frontend=frontend)
        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet'), self._config['pybullet'], gui=True)
        self.jsf = JointsStabilizedFilter(JSF_NUM_STEPS, JSF_DISTANCE_THRESHOLD)

        self._timestep = self._config['pybullet']['timestep']

        self._timer = rospy.Timer(rospy.Duration(1/UPDATE_RATE), self._update_cb)

    def _start_job(self, data):
        #print('\n\nSTARTING JOB')
        length = len(self._joint_names)
        waypoint = NodeParser(data['point'])
        self._target = waypoint.to_ros()
        self._joints = Joints(
            length=length,
            joint_names=self._joint_names,
            joint_positions=[0]*length,
            reachable=False)
        self._trace_data = {
            "lively_joint_names": list(self.ltk.joint_names),
            "lively_joint_data": {n:[] for n in self.ltk.joint_names},
            "lively_frame_names": list(self.ltk.frame_names),
            "lively_frame_data": {n:[] for n in self.ltk.frame_names},
            "pybullet_joint_names": list(self.pyb.joint_names),
            "pybullet_joint_data": {n:[] for n in self.pyb.joint_names},
            "pybullet_joint_velocities": {n:[] for n in self.pyb.joint_names},
            "pybullet_frame_names": list(self.pyb.frame_names),
            "pybullet_frame_data": {n:[] for n in self.pyb.frame_names},
            "pybullet_collisions": {}, #TODO fill this in later
            "pybullet_pinchpoints": {} #TODO fill this in later
        }

        self.jsf.clear()
        self.ltk.reset()

        self._updateCount = 0
        self._state = 'running'
        
    def _end_job(self, status, submit_fnt):
        #print('\n\nENDING JOB')

        self._state = 'idle'

        data = self._joints.to_dct() if status else None
        trace = self._trace_data

        self._target = None
        self._joints = None
        self._trace_data = None
        self._updateCount = 0

        #print(trace)

        submit_fnt(json.dumps({'joint': data, 'trace': trace}))

    def _update_cb(self, event=None):
        
        # If the job has been started
        # step toward target and record the joint positions
        # and when joints are stable (little/no-more optimization) then
        # check whether the target pose was reached within a margin of error
        if self._target != None and self._joints != None and self._trace_data != None and self._state == 'running':
            #print('\n\nUPDATE JOB')

            # run lively-ik, run pybullet model
            if self._target == None:
                return # Leave update if stop has been called

            (jp_ltk, jn_ltk), frames_ltk = self.ltk.step(self._target)
            ee_pose_ltk = LivelyTKSolver.get_ee_pose(frames_ltk[0])
            self.jsf.append(jp_ltk) # append to joint filter to know when lively is done
            pb_joints, pb_frames = self.pyb.step(jp_ltk, jn_ltk)

            if self._joints == None:
                return # leave update if stop has been called
            self._joints.set_joint_positions_by_names(jp_ltk,jn_ltk)

            poseWasReached = poseReached(self._target, ee_pose_ltk, POSITION_DISTANCE_THRESHOLD, ORIENTATION_DISTANCE_THRESHOLD)
            timeout = TIMEOUT_COUNT < self._updateCount
            if self.jsf.isStable() and (poseWasReached or timeout):

                #print('packing trace')

                # pack trace
                for n, p in zip(jn_ltk,jp_ltk):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['lively_joint_data'][n].append(p)

                (fp_ltk, fn_ltk) = frames_ltk
                for n, p in zip(fn_ltk, fp_ltk):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['lively_frame_data'][n].append(p)

                ee_pose_pyb = PyBulletModel.get_ee_pose(pb_frames)
                pb_collisions = self.pyb.collisionCheck()

                (jp_pby, jv_pby, jn_pby) = pb_joints
                for n, p, v in zip(jn_pby,jp_pby, jv_pby):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['pybullet_joint_data'][n].append(p)
                    self._trace_data['pybullet_joint_velocities'][n].append(p)

                (fp_pby, fn_pby) = pb_frames
                for n, p in zip(fn_pby, fp_pby):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['pybullet_frame_data'][n].append(p)

                #TODO collision packing & pinch point packing

                self._joints.reachability = poseReached(self._target, ee_pose_ltk, POSITION_DISTANCE_THRESHOLD, ORIENTATION_DISTANCE_THRESHOLD)
                self._job_queue.completed()
                self._state = 'idle'
            else:
                self._updateCount += 1

    def spin(self):
        rate = rospy.Rate(SPIN_RATE)
        while not rospy.is_shutdown():
            self._job_queue.update()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('joint_processor')

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")

    node = JointProcessor(config_path, config_file_name)
    node.spin()