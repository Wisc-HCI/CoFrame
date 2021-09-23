#!/usr/bin/env python3

'''
Converts abstract trajectories into executed robot movement traces.

Also applies graders to the traces to produce grade appraisals
'''

import os
import json
import rospy

from evd_script import Trace, NodeParser, Pose
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
        self._time_overall = 0
        self._time_step = 0
        self._index = 0
        self._input = None

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
        self._input = dct
        self._time_overall = 0
        self._time_step = 0
        self._path = []
        self._thresholds = []
        self._index = 0
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

        # structure the data to be captured (starts with initial state)
        self._trace_data = {
            "trajectory":dct[['trajectory']],
            "duration": 0,
            "time_data": [0],
            "type": self._type,
            "interpolator_path": {n: [None] for n in self.ltk.joint_names + ['ee_pose']},
            "lively_joint_names": self.ltk.joint_names,
            "lively_joint_data": {n:[locStart.joints.joint_positions[i]] for i, n in enumerate(self.ltk.joint_names)},
            "lively_frame_names": self.ltk.frame_names,
            "lively_frame_data": {n:[None] for n in self.ltk.frame_names},
            "pybullet_joint_names": self.pyb.joint_names,
            "pybullet_joint_data": {n:[None] for n in self.pyb.joint_names},
            "pybullet_joint_velocities": {n:[None] for n in self.pyb.joint_names},
            "pybullet_frame_names": self.pyb.frame_names,
            "pybullet_frame_data": {n:[None] for n in self.pyb.frame_names},
            "pybullet_collisions": {}, #TODO fill this in later
            "pybullet_pinchpoints": {} #TODO fill this in later
        }

        self.jsf.clear()
        self.ltk.set_joints(locStart.joints.joint_positions) # or jog to pose?
        self.pyb.set_joints(locStart.joints.joint_positions, locStart.joints.joint_names)

    def _end_job(self, status, submit_fnt):
        #trace = Trace.from_dct(self._trace_data)
        trace = self._trace_data
        inp = self._input

        self._path = None
        self._type = None
        self._thresholds = None
        self._trace_data = None
        self._input = None

        #data = trace.to_dct() if status else None
        submit_fnt(json.dumps({'input': inp, 'trace': json.dumps(trace)}))

    def _update_cb(self, event=None):
        # If the job has been started
        # step through trajectory and record the joints / frames as they occur
        if self._path != None and self._thresholds != None and self._type != None and self._trace_data != None:

            # self._path = [<interpolator>]
            # self._thresholds = [<reached_thresh>]
            # self._time_overall = <number>
            # self._time_step = <number>
            # self._trace_data = {data}
            # self._timestep = <number>
            # self._index = <number>
            if self._type == 'ee_ik':
                # interpolate pose in this leg of trajectory
                interpolator = self._path[self._index]
                ee_pose_itp = interpolator.step(self._time_step)

                self._trace_data['interpolator_path']['ee_pose'].append(Pose.from_ros(ee_pose_itp).to_simple_dct())

                # run lively-ik
                (jp_ltk, jn_ltk), frames_ltk = self.ltk.step(ee_pose_itp)
                ee_pose_ltk = LivelyTKSolver.get_ee_pose(frames_ltk[0])
                self.jsf.append(jp_ltk[0]) # append to joint filter to know when lively is done
                
                for n, p in zip(jn_ltk,jp_ltk):
                    self._trace_data['lively_joint_data'][n].append(p)

                (fp_ltk, fn_ltk) = frames_ltk
                for n, p in zip(fn_ltk, fp_ltk):
                    self._trace_data['lively_frame_data'][n].append(p)

                # run pybullet model
                pb_joints, pb_frames = self.pyb.step(jp_ltk, jn_ltk)
                ee_pose_pyb = PyBulletModel.get_ee_pose(pb_frames)
                pb_collisions = self.pyb.collisionCheck()

                (jp_pby, jv_pby, jn_pby) = pb_joints
                for n, p, v in zip(jn_pby,jp_pby, jv_pby):
                    self._trace_data['pybullet_joint_data'][n].append(p)
                    self._trace_data['pybullet_joint_velocities'][n].append(p)

                (fp_pby, fn_pby) = pb_frames
                for n, p in zip(fn_pby, fp_pby):
                    self._trace_data['pybullet_frame_data'][n].append(p)

                #TODO collision packing & pinch point packing
                
                # check if leg of trajectory is done
                (posThreshold, ortThreshold) = self._thresholds[self._index]
                if self.jsf.isStable() and poseReached(ee_pose_pyb, interpolator.end_pose, posThreshold, ortThreshold):
                    self._index += 1
                    self._time_step = 0
                else:
                    self._time_step += self._timestep

            else: # self._type == 'joint'
                pass #TODO implement this

            # record timing info
            self._time_overall += self._timestep
            self._trace_data['time_data'].append(self._time_overall)

            # end condition is when trajectory has been fully traced
            if self._index >= len(self._path):
                self._trace_data['duration'] = self._time_overall
                self._job_queue.completed()

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