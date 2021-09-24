#!/usr/bin/env python3

'''
Converts abstract trajectories into executed robot movement traces.

Also applies graders to the traces to produce grade appraisals
'''

import tf2_ros
import tf2_geometry_msgs
import os
import json
import rospy

from evd_script import Trace, NodeParser, Pose
from evd_interfaces.job_queue import JobQueue
from evd_sim.lively_tk_solver import LivelyTKSolver
from evd_sim.pybullet_model import PyBulletModel
from evd_sim.pose_reached import poseReached
from evd_sim.joints_reached import jointsReached
from evd_sim.pose_interpolator import PoseInterpolator
from evd_sim.joints_stabilized import JointsStabilizedFilter
from evd_interfaces.frontend_interface import FrontendInterface
from evd_sim.joint_interpolator import JointInterpolator


TIMEOUT_COUNT = 500
SPIN_RATE = 5
UPDATE_RATE = 1000
JSF_NUM_STEPS = 20
JSF_DISTANCE_THRESHOLD = 0.001

POSITION_INTERMEDIATE_DISTANCE_THRESHOLD = 0.1
ORIENTATION_INTERMEDIATE_DISTANCE_THRESHOLD = 0.5
POSITION_DISTANCE_THRESHOLD = 0.05
ORIENTATION_DISTANCE_THRESHOLD = 0.02
JOINTS_INTERMEDIATE_DISTANCE_THRESHOLD = 0.1
JOINTS_DISTANCE_THRESHOLD = 0.1


class TraceProcessor:

    def __init__(self, config_path, config_file_name, use_gui):
        self._path = None
        self._type = None
        self._thresholds = None
        self._trace_data = None
        self._time_overall = 0
        self._time_step = 0
        self._index = 0
        self._input = None
        self._state = 'idle'
        self._updateCount = 0

        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)

        self._fixed_frame = self._config['fixed_frame']
        self._ee_frame = self._config['link_groups']['end_effector_path']
        self._link_groups = self._config['link_groups']
        self._joint_names = self._config['joint_names']

        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        frontend = FrontendInterface(use_processor_configure=True)
        self._job_queue = JobQueue('trace', self._start_job, self._end_job, frontend=frontend)
        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet'), self._config['pybullet'], gui=use_gui)
        self.jsf = JointsStabilizedFilter(JSF_NUM_STEPS, JSF_DISTANCE_THRESHOLD)

        self._timestep = self._config['pybullet']['timestep']

        self._timer = rospy.Timer(rospy.Duration(1/UPDATE_RATE), self._update_cb)

    def _handle_pose_offset(self, point):
        transform = self._tf_buffer.lookup_transform(self._fixed_frame, point.link, rospy.Time(0), rospy.Duration(1.0))
        return tf2_geometry_msgs.do_transform_pose(point.to_ros(stamped=True), transform).pose

    def _handle_joint_packing(self, start, end):
        # Need to pivot
        joints = []
        for i in range(0,len(start)):
            joints.append([start[i], end[i]])
        return joints

    def _start_job(self, data):
        trajectory = NodeParser(data['trajectory'])
        points = {p['uuid']: NodeParser(p) for p in data['points']}

        # produce path from trajectory and points
        self._time_overall = 0
        self._time_step = 0
        self._path = []
        self._thresholds = []
        self._index = 0
        self._type = trajectory.move_type
        names = self.ltk.joint_names
        if self._type == 'joint':
            locStart = points[trajectory.start_location_uuid]
            lastJoints = locStart.joints.joint_positions

            for way in [points[id] for id in trajectory.waypoint_uuids]:
                nextJoints = way.joints.joint_positions
                self._path.append((JointInterpolator(self._handle_joint_packing(lastJoints, nextJoints), trajectory.velocity), names))
                lastJoints = nextJoints
                self._thresholds.append([JOINTS_INTERMEDIATE_DISTANCE_THRESHOLD]*len(nextJoints))
                
            locEnd = points[trajectory.end_location_uuid]
            nextJoints = locEnd.joints.joint_positions
            self._path.append((JointInterpolator(self._handle_joint_packing(lastJoints, nextJoints), trajectory.velocity), names))
            self._thresholds.append([JOINTS_DISTANCE_THRESHOLD]*len(nextJoints))

        else: # ee_ik
            locStart = points[trajectory.start_location_uuid]
            lastPose = self._handle_pose_offset(locStart)

            for way in [points[id] for id in trajectory.waypoint_uuids]:
                nextPose = self._handle_pose_offset(way)
                self._path.append(PoseInterpolator(lastPose, nextPose, trajectory.velocity))
                lastPose = nextPose
                self._thresholds.append((POSITION_INTERMEDIATE_DISTANCE_THRESHOLD,ORIENTATION_INTERMEDIATE_DISTANCE_THRESHOLD))

            locEnd = points[trajectory.end_location_uuid]
            self._path.append(PoseInterpolator(lastPose, self._handle_pose_offset(locEnd), trajectory.velocity))
            self._thresholds.append((POSITION_DISTANCE_THRESHOLD,ORIENTATION_DISTANCE_THRESHOLD))

        # structure the data to be captured (starts with initial state)
        self._trace_data = {
            "duration": 0,
            "time_data": [0],
            "type": self._type,
            "interpolator_path": {n: [None] for n in (self.ltk.joint_names + ['ee_pose'])},
            "lively_joint_names": list(self.ltk.joint_names),
            "lively_joint_data": {n:[locStart.joints.joint_positions[i]] for i, n in enumerate(self.ltk.joint_names)},
            "lively_frame_names": list(self.ltk.frame_names),
            "lively_frame_data": {n:[None] for n in self.ltk.frame_names},
            "pybullet_joint_names": list(self.ltk.joint_names),
            "pybullet_joint_data": {n:[None] for n in self.ltk.joint_names},
            "pybullet_joint_velocities": {n:[None] for n in self.ltk.joint_names},
            "pybullet_frame_names": list(self.pyb.frame_names),
            "pybullet_frame_data": {n:[None] for n in self.pyb.frame_names},
            "pybullet_collisions": {}, #TODO fill this in later
            "pybullet_occupancy": {},
            "pybullet_pinchpoints": {} #TODO fill this in later
        }

        self._input = data

        self.jsf.clear()
        self.ltk.set_joints(locStart.joints.joint_positions) # or jog to pose?
        self.pyb.set_joints(locStart.joints.joint_positions, locStart.joints.joint_names)

        self._updateCount = 0
        self._state = 'running'

    def _end_job(self, status, submit_fnt):
        self._state = 'idle'

        print('\n\n\nTrace Processor In End Job')
        # TODO need to find the error in json dump for ndim arrays
        print(self._trace_data['interpolator_path'])
        self._trace_data['interpolator_path'] = {}

        # List of problematic keys
        # - interpolator_path

        trace = self._trace_data
        inp = self._input

        self._path = None
        self._type = None
        self._thresholds = None
        self._trace_data = None
        self._input = None
        self._updateCount = 0

        #data = trace.to_dct() if status else None
        submit_fnt(json.dumps({'input': inp, 'trace': trace}))

    def _update_cb(self, event=None):
        # If the job has been started
        # step through trajectory and record the joints / frames as they occur
        if self._state == 'running':

            if self._type == 'ee_ik':
                # interpolate pose in this leg of trajectory
                interpolator = self._path[self._index]
                ee_pose_itp = interpolator.step(self._time_step)

                self._trace_data['interpolator_path']['ee_pose'].append(Pose.from_ros(ee_pose_itp).to_simple_dct())

                # run lively-ik
                (jp_ltk, jn_ltk), frames_ltk = self.ltk.step(ee_pose_itp)
                ee_pose_ltk = LivelyTKSolver.get_ee_pose(frames_ltk[0])
                self.jsf.append(jp_ltk) # append to joint filter to know when lively is done

                for n, p in zip(jn_ltk,jp_ltk):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['lively_joint_data'][n].append(p)

                (fp_ltk, fn_ltk) = frames_ltk
                for n, p in zip(fn_ltk, fp_ltk):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['lively_frame_data'][n].append(p)

                # run pybullet model
                pb_joints, pb_frames = self.pyb.step(jp_ltk, jn_ltk)
                ee_pose_pyb = PyBulletModel.get_ee_pose(pb_frames)

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
                pb_collisions = self.pyb.collisionCheck()
                pb_occupancy = self.pyb.occupancyCheck()
                pb_pinchs = self.pyb.pinchPointCheck()
                
                # check if leg of trajectory is done
                (posThreshold, ortThreshold) = self._thresholds[self._index]
                poseWasReached = poseReached(ee_pose_ltk, interpolator.end_pose, posThreshold, ortThreshold)
                inTimeout = TIMEOUT_COUNT < self._updateCount
                jointsAreStable = self.jsf.isStable()

                if jointsAreStable and (poseWasReached or inTimeout):
                    if inTimeout:
                        print('\n\n\nIN Timeout\n\n\n')
                        self._trace_data = None
                        self._job_queue.completed()
                        self._state = 'idle'
                        return # Failed to run trajectory

                    self._index += 1
                    self._time_step = 0
                    self._updateCount = 0
                else:
                    self._time_step += self._timestep
                    self._updateCount += 1

            else: # self._type == 'joint'
                # joint interpolation
                (interpolator, jn_itp) = self._path[self._index]
                jp_itp = interpolator.step(self._time_step)

                for n, j in zip(jn_itp, jp_itp):
                    self._trace_data['interpolator_path'][n].append(j)

                # run pybullet model
                pb_joints, pb_frames = self.pyb.step(jp_itp, jn_itp)
                ee_pose_pyb = PyBulletModel.get_ee_pose(pb_frames)

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
                pb_collisions = self.pyb.collisionCheck()
                pb_occupancy = self.pyb.occupancyCheck()
                pb_pinchs = self.pyb.pinchPointCheck()

                # check if leg of trajectoru is done
                joint_thresholds = self._thresholds[self._index]
                inTimeout = TIMEOUT_COUNT < self._updateCount
                jointsWasReached = jointsReached(jp_pby, interpolator.end_joints, joint_thresholds)
                if inTimeout or jointsWasReached:
                    if inTimeout:
                        print('\n\n\nIN Timeout\n\n\n')
                        self._trace_data = None
                        self._job_queue.completed()
                        self._state = 'idle'
                        return # Failed to run trajectory

                    self._index += 1
                    self._time_step = 0
                    self._updateCount = 0
                else:
                    self._time_step += self._timestep
                    self._updateCount += 1

            # record timing info
            self._time_overall += self._timestep
            self._trace_data['time_data'].append(self._time_overall)

            # end condition is when trajectory has been fully traced
            if self._index >= len(self._path):
                self._trace_data['duration'] = self._time_overall
                self._job_queue.completed()
                self._state = 'idle'

    def spin(self):
        rate = rospy.Rate(SPIN_RATE)
        while not rospy.is_shutdown():
            self._job_queue.update()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('trace_processor')

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")
    use_gui = rospy.get_param('~use_gui',False)

    node = TraceProcessor(config_path, config_file_name, use_gui)
    node.spin()