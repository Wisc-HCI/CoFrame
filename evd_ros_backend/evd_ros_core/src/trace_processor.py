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

POSITION_INTERMEDIATE_DISTANCE_THRESHOLD = 0.1
ORIENTATION_INTERMEDIATE_DISTANCE_THRESHOLD = 0.02

POSITION_DISTANCE_THRESHOLD = 0.05
ORIENTATION_DISTANCE_THRESHOLD = 0.02

JOINTS_INTERMEDIATE_DISTANCE_THRESHOLD = 0.1
JOINTS_DISTANCE_THRESHOLD = 0.1

class TraceProcessor:

    def __init__(self, config_path, config_file_name):
        self._path = None
        self._type = None
        self._thresholds = None
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

        self._timestep = self._config['pybullet']['timestep']

        self._timer = rospy.Timer(rospy.Duration(1/UPDATE_RATE), self._update_cb)

    def _start_job(self, data):
        dct = json.loads(data)
        trajectory = NodeParser(dct['trajectory'])
        points = {p['uuid']: NodeParser(p) for p in dct['points']}

        # produce path from trajectory and points
        self._path = []
        self._thresholds = []
        self._type = trajectory.move_type
        if self._type == 'joint':
            locStart = points[trajectory.start_location_uuid]
            lastJoints = locStart.joints.joint_positions

            for way in [points[id] for id in trajectory.waypoint_uuids]:
                self._path.append(JointInterpolator([lastJoints, way.joints.joint_positions], trajectory.velocity))
                lastJoints = way.joints.joint_positions
                self._thresholds.append([JOINTS_INTERMEDIATE_DISTANCE_THRESHOLD]*len(way.joints.joint_positions))
                
            locEnd = points[trajectory.end_location_uuid]
            self._path.append(JointInterpolator([lastJoints, locEnd.joints.joint_positions], trajectory.velocity))
            self._thresholds.append([JOINTS_DISTANCE_THRESHOLD]*len(locEnd.joints.joint_positions))

        else: # ee_ik
            locStart = points[trajectory.start_location_uuid]
            lastPose = locStart.to_ros()

            for way in [points[id] for id in trajectory.waypoint_uuids]:
                self._path.append(PoseInterpolator(lastPose, way.to_ros(), trajectory.velocity))
                lastPose = way.to_ros()
                self._thresholds.append((POSITION_INTERMEDIATE_DISTANCE_THRESHOLD,ORIENTATION_INTERMEDIATE_DISTANCE_THRESHOLD))

            locEnd = points[trajectory.end_location_uuid]
            self._path.append(PoseInterpolator(lastPose, locEnd.to_ros(), trajectory.velocity))
            self._thresholds.append((POSITION_DISTANCE_THRESHOLD,ORIENTATION_DISTANCE_THRESHOLD))

        # structure the data to be captured
        self._trace_data = {
            "time_data": [],
            "joint_names": self._joint_names,
            "joint_data": {key:[] for key in self._joint_names},
            "tf_data": {key:[] for key in [ self._link_groups['end_effector_path'] 
                                          + self._link_groups['joint_paths'] 
                                          + self._link_groups['tool_paths'] 
                                          + self._link_groups['component_paths']]},
            "grades": None,
            "duration": 0,
            "end_effector_path": self._link_groups['end_effector_path'],
            "joint_paths": self._link_groups['joint_paths'],
            "tool_paths": self._link_groups['tool_paths'],
            "component_paths": self._link_groups['component_paths']
        }

        self.jsf.clear()
        self.ltk.set_joints(locStart.joints.joint_positions) # or jog to pose?
        self.pyb.set_joints(locStart.joints.joint_positions, locStart.joints.joint_names)

    def _end_job(self, status, submit_fnt):
        trace = Trace.from_dct(self._trace_data)

        self._path = None
        self._type = None
        self._thresholds = None
        self._trace_data = None

        data = trace.to_dct() if status else None
        submit_fnt(json.dumps(data))

    def _update_cb(self):
        # If the job has been started
        # step through trajectory and record the joints / frames as they occur
        if self._path != None and self._thresholds != None and self._type != None and self._trace_data != None:
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