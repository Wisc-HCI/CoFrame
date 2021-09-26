#!/usr/bin/env python3

'''
Confirms reachability of waypoint or location. Generates a joint datastructure.
'''

import tf2_ros
import tf2_geometry_msgs
import os
import json
import rospy

from evd_sim.pose_reached import poseReached
from evd_interfaces.job_queue import JobQueue
from evd_sim.lively_tk_solver import LivelyTKSolver
from evd_sim.pybullet_model import PyBulletModel
from evd_sim.joints_stabilized import JointsStabilizedFilter
from evd_interfaces.frontend_interface import FrontendInterface
from evd_script import Joints, NodeParser, OccupancyZone, CollisionMesh


TIMEOUT_COUNT = 500
SPIN_RATE = 5
UPDATE_RATE = 1000
JSF_NUM_STEPS = 20
JSF_DISTANCE_THRESHOLD = 0.001
POSITION_DISTANCE_THRESHOLD = 0.05
ORIENTATION_DISTANCE_THRESHOLD = 0.02


class JointProcessor:
    
    def __init__(self, config_path, config_file_name, use_gui):
        self._input = None
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
        self._timestep = self._config['pybullet']['timestep']
        
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        frontend = FrontendInterface(use_processor_configure=True, processor_configure_cb=self._processor_configure_cb)
        self._job_queue = JobQueue('joints', self._start_job, self._end_job, frontend=frontend)
        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet'), self._config['pybullet'], gui=use_gui)
        self.jsf = JointsStabilizedFilter(JSF_NUM_STEPS, JSF_DISTANCE_THRESHOLD)

        self._timer = rospy.Timer(rospy.Duration(1/UPDATE_RATE), self._update_cb)

    def _processor_configure_cb(self, dct):
        self.pyb.registerCollisionMeshes(dct['collision_meshes'])
        self.pyb.registerOccupancyZones(dct['occupancy_zones'])

    def _start_job(self, data):
        #print('\n\nSTARTING JOB')
        length = len(self._joint_names)
        waypoint = NodeParser(data['point'])
        transform = self._tf_buffer.lookup_transform(self._fixed_frame, waypoint.link if waypoint.link != "" else "app", rospy.Time(0), rospy.Duration(1.0))
        self._target = tf2_geometry_msgs.do_transform_pose(waypoint.to_ros(stamped=True), transform).pose
        
        self._joints = Joints(
            length=length,
            joint_names=self._joint_names,
            joint_positions=[0]*length,
            reachable=False)

        self._trace_data = {
            "lively_joint_names": list(self.ltk.joint_names),
            "lively_joint_data": {n: None for n in self.ltk.joint_names},
            "lively_frame_names": list(self.ltk.frame_names),
            "lively_frame_data": {n: None for n in self.ltk.frame_names},
            "pybullet_joint_names": list(self.pyb.joint_names),
            "pybullet_joint_data": {n: None for n in self.pyb.joint_names},
            "pybullet_frame_names": list(self.pyb.frame_names),
            "pybullet_frame_data": {n: None for n in self.pyb.frame_names},
            "pybullet_collisions": {uuid: {n: None for n in self.pyb.frame_names} for uuid in self.pyb.collision_uuids}, 
            "pybullet_occupancy": {uuid: {n: None for n in self.pyb.frame_names} for uuid in self.pyb.occupancy_uuids},
            "pybullet_self_collisions": {n: {m: None for m in self.pyb.frame_names} for n in self.pyb.frame_names},
        }

        self._input = data

        self.jsf.clear()
        defaultJs, names = self.ltk.reset()
        self.pyb.set_joints(defaultJs, names)

        self._updateCount = 0
        self._state = 'running'
        
    def _end_job(self, status, submit_fnt):
        self._state = 'idle'

        data = self._joints.to_dct() if status else None
        trace = self._trace_data
        inp = self._input

        self._target = None
        self._joints = None
        self._input = None
        self._trace_data = None
        self._updateCount = 0

        submit_fnt(json.dumps({'input': inp, 'joint': data, 'trace': trace, 'status': status}))

    def _update_cb(self, event=None):
        
        # If the job has been started
        # step toward target and record the joint positions
        # and when joints are stable (little/no-more optimization) then
        # check whether the target pose was reached within a margin of error
        if self._state == 'running':
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
            inTimeout = TIMEOUT_COUNT < self._updateCount
            if self.jsf.isStable() and (poseWasReached or inTimeout):

                # pack trace
                for n, p in zip(jn_ltk,jp_ltk):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['lively_joint_data'][n] = p

                (fp_ltk, fn_ltk) = frames_ltk
                for n, p in zip(fn_ltk, fp_ltk):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['lively_frame_data'][n] = p

                ee_pose_pyb = PyBulletModel.get_ee_pose(pb_frames)

                (jp_pby, jv_pby, jn_pby) = pb_joints
                for n, p, v in zip(jn_pby,jp_pby, jv_pby):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['pybullet_joint_data'][n] = p

                (fp_pby, fn_pby) = pb_frames
                for n, p in zip(fn_pby, fp_pby):
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    self._trace_data['pybullet_frame_data'][n] = p

                # collision packing
                pb_collisions = self.pyb.collisionCheck()
                for uuid in pb_collisions.keys():
                    for frameName in pb_collisions[uuid].keys():
                        value = pb_collisions[uuid][frameName]
                        self._trace_data['pybullet_collisions'][uuid][frameName] = p

                pb_occupancy = self.pyb.occupancyCheck()
                for uuid in pb_occupancy.keys():
                    for frameName in pb_occupancy[uuid].keys():
                        value = pb_occupancy[uuid][frameName]
                        self._trace_data['pybullet_occupancy'][uuid][frameName] = p

                pb_selfCollisions = self.pyb.selfCollisionCheck()
                for a in pb_selfCollisions.keys():
                    for b in pb_selfCollisions[a].keys():
                        value = pb_selfCollisions[a][b]
                        self._trace_data['pybullet_self_collisions'][a][b] = p

                self._joints.reachable = poseWasReached
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
    use_gui = rospy.get_param('~use_gui',False)

    node = JointProcessor(config_path, config_file_name, use_gui)
    node.spin()