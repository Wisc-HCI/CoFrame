#!/usr/bin/env python3

import os
import math
import json
import rospy
import tf2_ros
import tf2_geometry_msgs

from evd_interfaces.job_queue import JobQueue
from evd_sim.joints_reached import jointsReached
from evd_sim.pybullet_model import PyBulletModel
from evd_sim.lively_tk_solver import LivelyTKSolver
from evd_sim.pose_interpolator import PoseInterpolator
from evd_sim.pinch_point_model import processPinchpoints
from evd_sim.joint_interpolator import JointInterpolator
from evd_sim.pose_reached import poseReached, deltaDebug
from evd_sim.joints_stabilized import JointsStabilizedFilter
from evd_interfaces.frontend_interface import FrontendInterface
from evd_script import Trace, NodeParser, Pose, OccupancyZone, CollisionMesh


SPIN_RATE = 5
UPDATE_RATE = 1000
JSF_NUM_STEPS = 10
JSF_DISTANCE_THRESHOLD = 0.01

POSITION_INTERMEDIATE_DISTANCE_THRESHOLD = 0.1
ORIENTATION_INTERMEDIATE_DISTANCE_THRESHOLD = 0.5
POSITION_DISTANCE_THRESHOLD = 0.05
ORIENTATION_DISTANCE_THRESHOLD = 0.02
JOINTS_INTERMEDIATE_DISTANCE_THRESHOLD = 0.1
JOINTS_DISTANCE_THRESHOLD = 0.1

JOINT_DISCONTINUITY_THRESHOLD = 1


class TraceProcessor:

    def __init__(self, config_path, config_file_name, use_gui):
        self._state = 'idle'
        self._active_job = None
        self._pending_configuration = None
        self._preempt_job = None
        self._ended_job = None
        
        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)

        self._fixed_frame = self._config['fixed_frame']
        self._ee_frame = self._config['link_groups']['end_effector_path']
        self._link_groups = self._config['link_groups']
        self._joint_names = self._config['joint_names']
        self._timestep = self._config['pybullet']['timestep']

        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet'), self._config['pybullet'], gui=use_gui)
        self.jsf = JointsStabilizedFilter(JSF_NUM_STEPS, JSF_DISTANCE_THRESHOLD)

        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        frontend = FrontendInterface(use_processor_configure=True, processor_configure_cb=self._processor_configure_cb)
        self._job_queue = JobQueue('trace', self._start_job_cb, self._end_job_cb, frontend=frontend)
        
        self._timer = rospy.Timer(rospy.Duration(1/UPDATE_RATE), self._update_cb)

    def _processor_configure_cb(self, dct):
        print('Trace Processor - Configure')
        self._pending_configuration = dct

    def _start_job_cb(self, data):
        print('Trace Processor - Starting Job')
        self._preempt_job = data

    def _end_job_cb(self, status, submit_fnt):
        if self._ended_job != None:
            print('Trace Processor - Ending Job')
            trace = self._ended_job['trace_data']
            inputData = self._ended_job['input']
            submit_fnt(json.dumps({'input': inputData, 'trace': trace, 'status': status}))
            self._ended_job = None
        else:
            submit_fnt(json.dumps({'status': status, 'trace': None, 'input': None}))

    def _update_cb(self, event=None):

        if self._state == 'idle' and self._pending_configuration != None:
            print('Trace Processor - Updating Configuration')
            config = self._pending_configuration
            self._pending_configuration = None
            self.register_objs(config)
            self._state = 'idle'

        elif self._preempt_job != None:
            print('Trace Processor - Preempting Job')
            job = self.pack_job(self._preempt_job)
            self._preempt_job = None
            self._ended_job = self._active_job

            if job == None:
                self._active_job = None
                self.job_queue.canceled()
                self._state = 'idle'
            else:
                self._active_job = job
                self.init_processing(job)
                self._state = 'running'

        elif self._state == 'running':
            job = self._active_job
            self.path_processing(job)
            if job['trace_data']['in_timeout'] or job['index'] >= len(job['path']):
                # if self.check_for_joint_discontinuity():
                #     self._state = 'joint_discontinuity_repair'
                # else:
                #     self._state = 'postprocessing'
                self._state = 'postprocessing' #TODO handle joint discontinuity

        elif self._state == 'joint_discontinuity_repair':
            print('Trace Processor - Joint Discontinuity Repair')
            job = self._active_job
            self.joint_discontinuity_repair(job)
            self._state = 'postprocessing' #TODO this needs to iterate

        elif self._state == 'postprocessing':
            print('Trace Processor - Post Processing')
            job = self._active_job
            job['trace_data']['duration'] = job['time_overall']
            self.post_processing(job)
            self._ended_job = job
            self._job_queue.completed()
            self._active_job = None
            self._state = 'idle'

        else:
            self._state = 'idle'

    def pack_job(self, data):
        try:
            trajectory = NodeParser(data['trajectory'])
            points = {p['uuid']: NodeParser(p) for p in data['points']}
        except:
            return None # ignoring this job

        time_overall = 0
        time_step = 0
        path = []
        thresholds = []
        index = 0
        mtype = trajectory.move_type
        names = self.ltk.joint_names
        expectedTime = 0

        locStart = points[trajectory.start_location_uuid]
        locEnd = points[trajectory.end_location_uuid]

        if mtype == 'joint':
            lastJoints = locStart.joints.joint_positions

            for way in [points[id] for id in trajectory.waypoint_uuids]:
                nextJoints = way.joints.joint_positions
                interp = JointInterpolator(self.handle_joint_packing(lastJoints, nextJoints), trajectory.velocity)
                expectedTime += interp.full_time
                path.append((interp, names))
                lastJoints = nextJoints
                thresholds.append([JOINTS_INTERMEDIATE_DISTANCE_THRESHOLD]*len(nextJoints))
                   
            nextJoints = locEnd.joints.joint_positions
            interp = JointInterpolator(self.handle_joint_packing(lastJoints, nextJoints), trajectory.velocity)
            expectedTime += interp.full_time
            path.append((interp, names))
            thresholds.append([JOINTS_DISTANCE_THRESHOLD]*len(nextJoints))
        
        elif mtype == 'ee_ik':
            lastPose = self.handle_pose_offset(locStart)

            for way in [points[id] for id in trajectory.waypoint_uuids]:
                nextPose = self.handle_pose_offset(way)
                interp = PoseInterpolator(lastPose, nextPose, trajectory.velocity)
                expectedTime += interp.full_time
                path.append((interp, way.joints.joint_positions))
                lastPose = nextPose
                thresholds.append((POSITION_INTERMEDIATE_DISTANCE_THRESHOLD,ORIENTATION_INTERMEDIATE_DISTANCE_THRESHOLD))

            nextPose = self.handle_pose_offset(locEnd)
            interp = PoseInterpolator(lastPose, nextPose, trajectory.velocity)
            expectedTime += interp.full_time
            path.append((interp, locEnd.joints.joint_positions))
            thresholds.append((POSITION_DISTANCE_THRESHOLD,ORIENTATION_DISTANCE_THRESHOLD))
        
        else:
            return # Ignoring job

        trace_data = {
            "type": mtype,
            "duration": 0,
            "estimated_duration": expectedTime * 2,
            "in_timeout": False,
            "time_data": [],

            "interpolator_path": {n: [] for n in self.ltk.joint_names} if mtype == 'joint' else {'ee_pose': []},

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
            "pybullet_collision_frame_names": list(self.pyb.collision_frame_names),
            "pybullet_occupancy_frame_names": list(self.pyb.occupancy_frame_names),
            "pybullet_self_collision_filter": self.pyb.self_collision_filter,
            
            "pybullet_collisions": {uuid: {n: [] for n in self.pyb.collision_frame_names} for uuid in self.pyb.collision_uuids}, 
            "pybullet_occupancy": {uuid: {n: [] for n in self.pyb.occupancy_frame_names} for uuid in self.pyb.occupancy_uuids},
            "pybullet_self_collisions": {n: {m: [] for m in self.pyb.collision_frame_names} for n in self.pyb.collision_frame_names},

            "postprocess_collisions": {n: [] for n in self.pyb.collision_frame_names},
            "postprocess_collisions_objs": {n: [] for n in self.pyb.collision_frame_names},
            "postprocess_self_collisions": {n: [] for n in self.pyb.collision_frame_names},
            "postprocess_self_collisions_objs": {n: [] for n in self.pyb.collision_frame_names},
            "postprocess_occupancy": {n: [] for n in self.pyb.occupancy_frame_names},
            "postprocess_occupancy_objs": {n: [] for n in self.pyb.occupancy_frame_names},

            "pinchpoints_tracks": None,
            "pinchpoints_semantics": None
        }

        return {
            'trace_data': trace_data,
            'path': path,
            'mtype': mtype,
            'thresholds': thresholds,
            'time_overall': time_overall,
            'time_step': time_step,
            'index': index,
            'input': data,
            'starting_joints': (locStart.joints.joint_positions, locStart.joints.joint_names),
            'ending_joints': (locEnd.joints.joint_positions, locEnd.joints.joint_names)
        }

    def init_processing(self, job):
        (jp, jn) = job['starting_joints']
        self.jsf.clear()
        self.ltk.set_joints(jp)
        self.pyb.set_joints(jp, jn)

    def path_processing(self, job):

        currentState = None
        targetState = None
        jointPositionsFromInterp = None
        jointNamesFromInterp = None
        
        # Interpolation -> Generate joints for pybullet
        if job['mtype'] == 'ee_ik':
            (interpolator, tJoints) = job['path'][job['index']]
            ee_pose_itp = interpolator.step(job['time_step'])

            job['trace_data']['interpolator_path']['ee_pose'].append(Pose.from_ros(ee_pose_itp).to_simple_dct())

            (jp_ltk, jn_ltk), frames_ltk = self.ltk.step(ee_pose_itp, finalJoints=tJoints)
            ee_pose_ltk = LivelyTKSolver.get_ee_pose(frames_ltk[0])

            for n, p in zip(jn_ltk,jp_ltk):
                job['trace_data']['lively_joint_data'][n].append(p)

            (fp_ltk, fn_ltk) = frames_ltk
            for n, p in zip(fn_ltk, fp_ltk):
                job['trace_data']['lively_frame_data'][n].append(p)

            currentState = ee_pose_ltk
            targetState = interpolator.end_pose
            jointPositionsFromInterp = jp_ltk
            jointNamesFromInterp = jn_ltk

        elif job['mtype'] == 'joint':
            (interpolator, jn_itp) = job['path'][job['index']]
            jp_itp = interpolator.step(job['time_step'])

            for n, j in zip(jn_itp, jp_itp):
                job['trace_data']['interpolator_path'][n].append(float(j))

            currentState = jp_itp
            targetState = interpolator.end_joints
            jointPositionsFromInterp = jp_itp
            jointNamesFromInterp = jn_itp
        
        else:
            print('wtf?')

        # Feed joint stabilzier
        self.jsf.append(jointPositionsFromInterp)

        # Execute pybullet model
        self.run_pybullet(job, jointPositionsFromInterp, jointNamesFromInterp)
        self.pack_collisions(job)

        # check if leg of trajectory is done
        inTimeout = self.check_in_timeout(job)
        jointsStable = self.jsf.isStable()
        targetReached = self.reached_target(job, currentState, targetState)

        print('\t Running', 'T~', inTimeout, 'or ( JS~', jointsStable, 'and R~', targetReached, ')')

        if (jointsStable and targetReached) or inTimeout:
            job['index'] += 1
            job['time_step'] = 0
        else:
            job['time_step'] += self._timestep

        job['time_overall'] += self._timestep
        job['trace_data']["in_timeout"] = inTimeout
        job['trace_data']['time_data'].append(job['time_overall'])

    def joint_discontinuity_repair(self, job):
        pass #TODO

    def post_processing(self, job):
        self.pivot_collisions(job)
        self.pivot_occupancy(job)
        self.pivot_self_collision(job)
        self.compute_pinch_points(job)
 
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

    def handle_pose_offset(self, point):
        transform = self._tf_buffer.lookup_transform(self._fixed_frame, point.link if point.link != "" else "app", rospy.Time(0), rospy.Duration(1.0))
        return tf2_geometry_msgs.do_transform_pose(point.to_ros(stamped=True), transform).pose

    def handle_joint_packing(self, start, end):
        joints = []
        for i in range(0,len(start)):
            joints.append([start[i], end[i]])
        return joints

    def register_objs(self, dct):
        self.pyb.registerCollisionMeshes(dct['collision_meshes'])
        self.pyb.registerOccupancyZones(dct['occupancy_zones'])

    def check_for_joint_discontinuity(self, joints_a, joints_b):
        for idx in range(0,len(joints_a)):
            if abs(joints_a[idx] - joints_b[idx]) > JOINT_DISCONTINUITY_THRESHOLD:
                return True

        return False

    def run_pybullet(self, job, jointPositionsFromInterp, jointNamesFromInterp):
        # Run pybullet
        pb_joints, pb_frames = self.pyb.step(jointPositionsFromInterp, jointNamesFromInterp)

        (jp_pby, jv_pby, jn_pby) = pb_joints
        for n, p, v in zip(jn_pby,jp_pby, jv_pby):
            job['trace_data']['pybullet_joint_data'][n].append(p)

        (fp_pby, fn_pby) = pb_frames
        for n, p in zip(fn_pby, fp_pby):
            job['trace_data']['pybullet_frame_data'][n].append(p)

    def pack_collisions(self, job):
        # collision packing
        pb_collisions = self.pyb.collisionCheck()
        for uuid in pb_collisions.keys():
            for frameName in pb_collisions[uuid].keys():
                value = pb_collisions[uuid][frameName]
                job['trace_data']['pybullet_collisions'][uuid][frameName].append(value)

        pb_selfCollisions = self.pyb.selfCollisionCheck()
        for a in pb_selfCollisions.keys():
            for b in pb_selfCollisions[a].keys():
                value = pb_selfCollisions[a][b]
                job['trace_data']['pybullet_self_collisions'][a][b].append(value)

        pb_occupancy = self.pyb.occupancyCheck()
        for uuid in pb_occupancy.keys():
            for frameName in pb_occupancy[uuid].keys():
                value = pb_occupancy[uuid][frameName]
                job['trace_data']['pybullet_occupancy'][uuid][frameName].append(value)

    def reached_target(self, job, currentState, targetState):
        reachedTarget = False
        if job['mtype'] == 'ee_ik':
            (posThreshold, ortThreshold) = job['thresholds'][job['index']]
            reachedTarget = poseReached(currentState, targetState, posThreshold, ortThreshold)
        elif job['mtype'] == 'joint':
            joint_thresholds = job['thresholds'][job['index']]
            reachedTarget = jointsReached(currentState, targetState, joint_thresholds)
        else: # WTF?
            print('wtf')

        return reachedTarget

    def check_in_timeout(self, job):
        return job['time_overall'] > job['trace_data']["estimated_duration"] * 2

    def pivot_collisions(self, job):
        # Pivot collisions to be robot frames first
        # For each frame provide a float [0 to 1] for each step in time for closest collision.
        # 0 means no collision, 1 means in collision, in between means approaching collision
        # We need to set an arbitrary distance for this
        for n in job['trace_data']["pybullet_collision_frame_names"]:
            for idx in range(0,len(job['trace_data']['time_data'])):
                min_dist = float('inf')
                obj = None

                for uuid in job['trace_data']["pybullet_collisions"].keys():
                    #print('Time_Data',len(job['trace_data']['time_data']), uuid, n, len(job['trace_data']["pybullet_collisions"][uuid][n]))
                    if job['trace_data']["pybullet_collisions"][uuid][n][idx] != None:
                        dist = job['trace_data']["pybullet_collisions"][uuid][n][idx]['distance']
                        if dist < min_dist:
                            min_dist = dist
                            obj = uuid

                value = self.collision_mapping(min_dist, 0.05)
                if value <= 0:
                    obj = None # If no collision detected

                job['trace_data']["postprocess_collisions"][n].append(value)
                job['trace_data']["postprocess_collisions_objs"][n].append(obj)

    def pivot_occupancy(self, job):
        for n in job['trace_data']["pybullet_occupancy_frame_names"]:
            for idx in range(0,len(job['trace_data']['time_data'])):
                min_dist = float('inf')
                obj = None

                for uuid in job['trace_data']["pybullet_occupancy"].keys():
                    if job['trace_data']["pybullet_occupancy"][uuid][n][idx] != None:
                        dist = job['trace_data']["pybullet_occupancy"][uuid][n][idx]['distance']
                        if dist < min_dist:
                            min_dist = dist
                            obj = uuid

                value = self.collision_mapping(min_dist, 0.05)
                if value <= 0:
                    obj = None # If no collision detected

                job['trace_data']["postprocess_occupancy"][n].append(value)
                job['trace_data']["postprocess_occupancy_objs"][n].append(obj)

    def pivot_self_collision(self, job):
        # we might need to filter this a bit (right now neighbors will always collide)
        for n in job['trace_data']["pybullet_self_collisions"].keys():
            for idx in range(0,len(job['trace_data']["time_data"])):
                min_dist = float('inf')
                obj = None

                for m in job['trace_data']["pybullet_self_collisions"][n].keys():
                    if job['trace_data']["pybullet_self_collisions"][n][m][idx] != None:
                        dist = job['trace_data']["pybullet_self_collisions"][n][m][idx]['distance']
                        if dist < min_dist:
                            min_dist = dist
                            obj = m
                
                value = self.collision_mapping(min_dist, 0.05)
                if value <= 0:
                    obj = None # If no collision detected
                
                job['trace_data']["postprocess_self_collisions"][n].append(value)
                job['trace_data']["postprocess_self_collisions_objs"][n].append(obj)

    def compute_pinch_points(self, job):
        # Calculate pinch points
        tracks, semantics = processPinchpoints(job['trace_data']["pybullet_self_collisions"], len(job['trace_data']["time_data"]))
        job['trace_data']["pinchpoints_tracks"] = tracks
        job['trace_data']["pinchpoints_semantics"] = semantics


if __name__ == "__main__":
    rospy.init_node('trace_processor')

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")
    use_gui = rospy.get_param('~use_gui',False)

    node = TraceProcessor(config_path, config_file_name, use_gui)
    node.spin()