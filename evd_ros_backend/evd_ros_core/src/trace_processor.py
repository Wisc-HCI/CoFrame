#!/usr/bin/env python3

'''
Converts abstract trajectories into executed robot movement traces.

Also applies graders to the traces to produce grade appraisals
'''

import os
import json
import rospy
import tf2_ros
import tf2_geometry_msgs

from evd_sim.pose_reached import poseReached
from evd_interfaces.job_queue import JobQueue
from evd_sim.joints_reached import jointsReached
from evd_sim.pybullet_model import PyBulletModel
from evd_sim.lively_tk_solver import LivelyTKSolver
from evd_sim.pose_interpolator import PoseInterpolator
from evd_sim.joint_interpolator import JointInterpolator
from evd_sim.joints_stabilized import JointsStabilizedFilter
from evd_interfaces.frontend_interface import FrontendInterface
from evd_script import Trace, NodeParser, Pose, OccupancyZone, CollisionMesh


TIMEOUT_COUNT = 500
SPIN_RATE = 5
UPDATE_RATE = 1000
JSF_NUM_STEPS = 10
JSF_DISTANCE_THRESHOLD = 0.005

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
        self._pending_config = None

        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)

        self._fixed_frame = self._config['fixed_frame']
        self._ee_frame = self._config['link_groups']['end_effector_path']
        self._link_groups = self._config['link_groups']
        self._joint_names = self._config['joint_names']
        self._timestep = self._config['pybullet']['timestep']

        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        frontend = FrontendInterface(use_processor_configure=True, processor_configure_cb=self._processor_configure_cb)
        self._job_queue = JobQueue('trace', self._start_job, self._end_job, frontend=frontend)
        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet'), self._config['pybullet'], gui=use_gui)
        self.jsf = JointsStabilizedFilter(JSF_NUM_STEPS, JSF_DISTANCE_THRESHOLD)

        self._timer = rospy.Timer(rospy.Duration(1/UPDATE_RATE), self._update_cb)

    def _processor_configure_cb(self, dct):
        print('Trace Processor - Configure', dct, 'Current State', self._state)

        if self._state != 'idle':
            print('pending')
            self._pending_config = dct
        else:    
            print('now')
            self.pyb.registerCollisionMeshes(dct['collision_meshes'])
            self.pyb.registerOccupancyZones(dct['occupancy_zones'])

    def _handle_pose_offset(self, point):
        transform = self._tf_buffer.lookup_transform(self._fixed_frame, point.link if point.link != "" else "app", rospy.Time(0), rospy.Duration(1.0))
        return tf2_geometry_msgs.do_transform_pose(point.to_ros(stamped=True), transform).pose

    def _handle_joint_packing(self, start, end):
        # Need to pivot
        joints = []
        for i in range(0,len(start)):
            joints.append([start[i], end[i]])
        return joints

    def _start_job(self, data):
        self._state = 'starting'

        print('Trace Processor - Starting Job', data['trajectory']['uuid'])
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
            "type": self._type,
            "duration": 0,
            "time_data": [],

            "interpolator_path": {n: [] for n in self.ltk.joint_names} if self._type == 'joint' else {'ee_pose': []},

            "lively_joint_names": list(self.ltk.joint_names),
            "lively_joint_data": {n: [] for i, n in enumerate(self.ltk.joint_names)},
            "lively_frame_names": list(self.ltk.frame_names),
            "lively_frame_data": {n:[] for n in self.ltk.frame_names},

            "pybullet_joint_names": list(self.ltk.joint_names),
            "pybullet_joint_data": {n:[] for i, n in enumerate(self.ltk.joint_names)},
            "pybullet_frame_names": list(self.pyb.frame_names),
            "pybullet_frame_data": {n:[] for n in self.pyb.frame_names},
            "pybullet_collision_uuids": list(self.pyb.collision_uuids),
            "pybullet_occupancy_uuids": list(self.pyb.occupancy_uuids),
            "pybullet_collisions": {uuid: {n: [] for n in self.pyb.frame_names} for uuid in self.pyb.collision_uuids}, 
            "pybullet_occupancy": {uuid: {n: [] for n in self.pyb.frame_names} for uuid in self.pyb.occupancy_uuids},
            "pybullet_self_collisions": {n: {m: [] for m in self.pyb.frame_names} for n in self.pyb.frame_names},

            "postprocess_collisions": {n: [] for n in self.pyb.frame_names},
            "postprocess_collisions_objs": {n: [] for n in self.pyb.frame_names},
            "postprocess_self_collisions": {n: [] for n in self.pyb.frame_names},
            "postprocess_self_collisions_objs": {n: [] for n in self.pyb.frame_names},
            "postprocess_occupancy": {n: [] for n in self.pyb.frame_names},
            "postprocess_occupancy_objs": {n: [] for n in self.pyb.frame_names}
        }

        self._input = data

        self.jsf.clear()
        self.ltk.set_joints(locStart.joints.joint_positions) # or jog to pose?
        self.pyb.set_joints(locStart.joints.joint_positions, locStart.joints.joint_names)

        self._updateCount = 0
        self._state = 'running'

    def _end_job(self, status, submit_fnt):
        self._state = 'ending'

        print('Trace Processor Ending Job', self._input['trajectory']['uuid'])
        trace = self._trace_data
        inp = self._input

        self._path = None
        self._type = None
        self._thresholds = None
        self._trace_data = None
        self._input = None
        self._updateCount = 0

        if self._pending_config != None:
            self.pyb.registerCollisionMeshes(self._pending_config['collision_meshes'])
            self.pyb.registerOccupancyZones(self._pending_config['occupancy_zones'])
            self._pending_config = None

        #data = trace.to_dct() if status else None
        submit_fnt(json.dumps({'input': inp, 'trace': trace, 'status': status}))
        self._state = 'idle'

    def _update_cb(self, event=None):
        # If the job has been started
        # "running" - step through trajectory and record the joints / frames as they occur
        # "post" - transform data for frontend
        if self._state == 'running':

            self.__raw_processing()
            
            # record timing info
            self._time_overall += self._timestep
            if self._trace_data == None:
                return # leave update if stop has been called
            self._trace_data['time_data'].append(self._time_overall)

            # end condition is when trajectory has been fully traced
            if self._index >= len(self._path):
                if self._trace_data == None:
                    return # leave update if stop has been called
                self._trace_data['duration'] = self._time_overall
                #self._job_queue.completed()
                self._state = 'post'

        elif self._state == 'post':

            self.__post_processing()

            self._job_queue.completed()
            self._state = 'idle'

    def __raw_processing(self):
        print('Trace Processor - Update', self._type, self._index, self._time_step, self._updateCount)

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

            (fp_pby, fn_pby) = pb_frames
            for n, p in zip(fn_pby, fp_pby):
                if self._trace_data == None:
                    return # leave update if stop has been called
                self._trace_data['pybullet_frame_data'][n].append(p)

            # collision packing
            pb_collisions = self.pyb.collisionCheck()
            for uuid in pb_collisions.keys():
                for frameName in pb_collisions[uuid].keys():
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    if uuid not in self._trace_data['pybullet_collision_uuids']:
                        break # configuration changed
                        
                    value = pb_collisions[uuid][frameName]
                    self._trace_data['pybullet_collisions'][uuid][frameName].append(value)

            pb_occupancy = self.pyb.occupancyCheck()
            for uuid in pb_occupancy.keys():
                for frameName in pb_occupancy[uuid].keys():
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    if uuid not in self._trace_data['pybullet_occupancy_uuids']:
                        break # configuration changed

                    value = pb_occupancy[uuid][frameName]
                    self._trace_data['pybullet_occupancy'][uuid][frameName].append(value)

            pb_selfCollisions = self.pyb.selfCollisionCheck()
            for a in pb_selfCollisions.keys():
                for b in pb_selfCollisions[a].keys():
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    value = pb_selfCollisions[a][b]
                    self._trace_data['pybullet_self_collisions'][a][b].append(value)
            
            # check if leg of trajectory is done
            (posThreshold, ortThreshold) = self._thresholds[self._index]
            poseWasReached = poseReached(ee_pose_ltk, interpolator.end_pose, posThreshold, ortThreshold)
            inTimeout = TIMEOUT_COUNT < self._updateCount
            jointsAreStable = self.jsf.isStable()

            print('Joints stable', jointsAreStable, 'pose reached', poseWasReached, 'in timeout', inTimeout)

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
                self._trace_data['interpolator_path'][n].append(float(j))

            # run pybullet model
            pb_joints, pb_frames = self.pyb.step(jp_itp, jn_itp)
            ee_pose_pyb = PyBulletModel.get_ee_pose(pb_frames)

            (jp_pby, jv_pby, jn_pby) = pb_joints
            for n, p, v in zip(jn_pby,jp_pby, jv_pby):
                if self._trace_data == None:
                    return # leave update if stop has been called
                self._trace_data['pybullet_joint_data'][n].append(p)

            (fp_pby, fn_pby) = pb_frames
            for n, p in zip(fn_pby, fp_pby):
                if self._trace_data == None:
                    return # leave update if stop has been called
                self._trace_data['pybullet_frame_data'][n].append(p)

            # collision packing
            pb_collisions = self.pyb.collisionCheck()
            for uuid in pb_collisions.keys():
                for frameName in pb_collisions[uuid].keys():
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    if uuid not in self._trace_data['pybullet_collision_uuids']:
                        break # configuration changed

                    value = pb_collisions[uuid][frameName]
                    self._trace_data['pybullet_collisions'][uuid][frameName].append(value)

            pb_occupancy = self.pyb.occupancyCheck()
            for uuid in pb_occupancy.keys():
                for frameName in pb_occupancy[uuid].keys():
                    if self._trace_data == None:
                        return # leave update if stop has been called
                    if uuid not in self._trace_data['pybullet_occupancy_uuids']:
                        break # configuration changed

                    value = pb_occupancy[uuid][frameName]
                    self._trace_data['pybullet_occupancy'][uuid][frameName].append(value)

            pb_selfCollisions = self.pyb.selfCollisionCheck()
            for a in pb_selfCollisions.keys():
                for b in pb_selfCollisions[a].keys():
                    if self._trace_data == None:
                        return # leave update if stop has been called

                    value = pb_selfCollisions[a][b]
                    self._trace_data['pybullet_self_collisions'][a][b].append(value)

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

    def __post_processing(self):

        print('Trace Processor - Post Processing')

        # Pivot collisions to be robot frames first
        # For each frame provide a float [0 to 1] for each step in time for closest collision.
        # 0 means no collision, 1 means in collision, in between means approaching collision
        # We need to set an arbitrary distance for this
        for n in self._trace_data["pybullet_frame_names"]:
            for idx in range(0,len(self._trace_data['time_data'])):

                min_dist = float('inf')
                obj = None
                for uuid in self._trace_data["pybullet_collisions"].keys():
                    if self._trace_data["pybullet_collisions"][uuid][n][idx] != None:
                        dist = self._trace_data["pybullet_collisions"][uuid][n][idx]['distance']
                        if dist < min_dist:
                            min_dist = dist
                            obj = uuid

                value = self.collision_mapping(min_dist, 0.05)
                if value <= 0:
                    obj = None # If no collision detected

                self._trace_data["postprocess_collisions"][n].append(value)
                self._trace_data["postprocess_collisions_objs"][n].append(obj)

        # Pivot self collisions
        #TODO we might need to filter this a bit (right now neighbors will always collide)
        for n in self._trace_data["pybullet_self_collisions"].keys():
            for idx in range(0,len(self._trace_data["time_data"])):

                min_dist = float('inf')
                obj = None
                for m in self._trace_data["pybullet_self_collisions"][n].keys():
                    if self._trace_data["pybullet_self_collisions"][n][m][idx] != None:
                        dist = self._trace_data["pybullet_self_collisions"][n][m][idx]['distance']
                        if dist < min_dist:
                            min_dist = dist
                            obj = m
                
                value = self.collision_mapping(min_dist, 0.05)
                if value <= 0:
                    obj = None # If no collision detected
                
                self._trace_data["postprocess_self_collisions"][n].append(value)
                self._trace_data["postprocess_self_collisions_objs"][n].append(obj)

        # Pivot occupancy zones
        for n in self._trace_data["pybullet_frame_names"]:
            for idx in range(0,len(self._trace_data['time_data'])):

                min_dist = float('inf')
                obj = None
                for uuid in self._trace_data["pybullet_occupancy"].keys():
                    if self._trace_data["pybullet_occupancy"][uuid][n][idx] != None:
                        dist = self._trace_data["pybullet_occupancy"][uuid][n][idx]['distance']
                        if dist < min_dist:
                            min_dist = dist
                            obj = uuid

                value = self.collision_mapping(min_dist, 0.05)
                if value <= 0:
                    obj = None # If no collision detected

                self._trace_data["postprocess_occupancy"][n].append(value)
                self._trace_data["postprocess_occupancy_objs"][n].append(obj)

    def spin(self):
        rate = rospy.Rate(SPIN_RATE)
        while not rospy.is_shutdown():
            self._job_queue.update()
            rate.sleep()

    def clamp(self, v, minV=0, maxV=1):
        if v < minV:
            return minV
        elif v > maxV:
            return maxV
        else:
            return v

    def collision_mapping(self, dist, threshold):
        return self.clamp((-1 / threshold) * dist + 1)


if __name__ == "__main__":
    rospy.init_node('trace_processor')

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")
    use_gui = rospy.get_param('~use_gui',False)

    node = TraceProcessor(config_path, config_file_name, use_gui)
    node.spin()