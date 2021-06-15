#!/usr/bin/env python3

'''
Trace processor generates traces from trajectories using
- RelaxedIK

Traces trajectories using
- URSim

Creates full visualizations for trajectories
- end-effector-path
- occupancy volume mesh
- joint-link-paths (more general form of end-effector path)

Generates waypoint joints
'''

#TODO Planner integration

import tf
import json
import time
import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from evd_ros_core.msg import RobotStop, RobotServo, RobotMove
from evd_ros_core.msg import RobotMoveTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point
from evd_ros_core.srv import SetPendingJobs, SetPendingJobsRequest, SetPendingJobsResponse
from evd_ros_core.srv import GetPendingJobs, GetPendingJobsRequest, GetPendingJobsResponse

from evd_interfaces.robot_interface import RobotInterface

from evd_script.data_nodes.trace import Trace
from evd_script.data_nodes.trajectory import Trajectory
from evd_script.data_nodes.waypoint import Waypoint
from evd_script.data_nodes.location import Location
from evd_script.data_nodes.geometry import Pose, Position, Orientation


class TraceProcessor:

    def __init__(self):
        self._waypoint_jobs = []
        self._trace_jobs = []
        self._tf_samples = {}
        self._js_samples = {}
        self._current_js_state = None
        self._fixed_frame = '/planner_base_link'

        self._link_groups = {
            'end_effector_path': 'planner_ee_link',
            'joint_paths': [
                'planner_base_link',
                'planner_shoulder_link',
                'planner_upper_arm_link',
                'planner_forearm_link',
                'planner_wrist_1_link',
                'planner_wrist_2_link',
                'planner_wrist_3_link',
            ],
            'tool_paths': [
                'planner_robotiq_85_left_finger_tip_link',
                'planner_robotiq_85_right_finger_tip_link'
            ],
            'component_paths': []
        }

        self._joints = [
            'planner_shoulder_pan_joint',
            'planner_shoulder_lift_joint',
            'planner_elbow_joint',
            'planner_wrist_1_joint',
            'planner_wrist_2_joint',
            'planner_wrist_3_joint'
        ]

        self._ursim = RobotInterface('planner')
        self._listener = tf.TransformListener()
        self._js_sub = rospy.Subscriber('planner/joint_states_labeled',JointState,self._js_cb)
        self._data_client = DataClientInterface(on_program_update_cb=self._program_updated)
        self._issue_client = IssueClientInterface()

    def _program_updated(self):

        #create a list of jobs
        # - all waypoints not assigned joints need to be tested
        waypoints = self._data_client.cache.waypoints.values()
        new_uuids = set([w.uuid for w in waypoints if w.joints == None or len(w.joints) == 0])
        old_uuids = set([w['uuid'] for w in self._waypoint_jobs])

        update = new_uuids & old_uuids
        remove = old_uuids - update
        append = new_uuids - update

        for uuid in remove:
            for job in self._waypoint_jobs:
                if job['uuid'] == uuid:
                    job['remove'] = True
                    self._issue_client.clear_pending_job('trace_processor',uuid)
                    break
        self._waypoint_jobs = [j for j in self._waypoint_jobs if 'remove' not in j.keys()]

        for uuid in update:
            for job in self._waypoint_jobs:
                if job['uuid'] == uuid:
                    job['data'] = self._data_client.cache.get(uuid,'waypoint')
                    break

        for uuid in append:
            self._waypoint_jobs.append({'uuid': uuid, 'data': self._data_client.cache.get(uuid,'waypoint')})

        self._issue_client.set_pending_jobs(
            source='trace_processor',
            ids=append,
            human_messages=['Generating waypoint joints']*len(append),
            data=[json.dumps(self._data_client.cache.get(uuid,'waypoint').to_dct()) for uuid in append])

        # - then run all traces
        trajectories = self._data_client.cache.trajectories.values()
        new_uuids = set([t.uuid for t in trajectories if t.trace == None])
        old_uuids = set([t['uuid'] for t in self._trace_jobs])

        update = new_uuids & old_uuids
        remove = old_uuids - update
        append = new_uuids - update

        for uuid in remove:
            for job in self._trace_jobs:
                if job['uuid'] == uuid:
                    job['remove'] = True
                    self._issue_client.clear_pending_job('trace_processor',uuid)
                    break
        self._trace_jobs = [j for j in self._trace_jobs if 'remove' not in j.keys()]

        for uuid in update:
            for job in self._trace_jobs:
                if job['uuid'] == uuid:
                    job['data'] = self._data_client.cache.get(uuid,'trajectory')
                    break

        for uuid in append:
            self._trace_jobs.append({'uuid': uuid, 'data': self._data_client.cache.get(uuid,'trajectory')})

        self._issue_client.set_pending_jobs(
            source='trace_processor',
            ids=append,
            human_messages=['Generating trace']*len(append),
            data=[json.dumps(self._data_client.cache.get(uuid,'trajectory').to_dct()) for uuid in append])

    def _js_cb(self, msg):
        if msg != None:
            self._current_js_state = msg

    def generate_waypoint(self, evd_waypoint): # and location

        # set state of robot
        print('setting state of robot')
        pose = evd_waypoint.to_ros()
        _m, result = self.set_robot_pose(pose)

        # get and save joint state
        joint_vals = [0]*len(self._joints)
        for i in range(0,len(self._current_js_state.name)):
            if self._current_js_state.name[i] in self._joints:
                idx = self._joints.index(self._current_js_state.name[i])
                joint_vals[idx] = self._current_js_state.position[i]

        evd_waypoint.joints = joint_vals

    def generate_trace(self, evd_trajectory):

        # generate trajectory path for robot
        print('generating full trajectory path')
        fullTraj = self.pack_robot_trajectory(evd_trajectory)

        # reset state of robot to initial joint state
        print('resetting state of robot')
        startLoc = evd_trajectory.context.get_location(evd_trajectory.start_location_uuid)
        self.set_robot_joints(startLoc.joints)

        # run trace, collecting links of interest at sample frequency
        print('running full trajectory and capturing trace data')
        self._tf_samples = {group:[] for group in self._link_groups.keys()}

        self.start_sample_timer()
        start_time = time.time()
        result = self.perform_robot_trajectory(fullTraj)
        end_time = time.time()
        self.RobotStop_sample_timer()

        timeVal = end_time - start_time
        print(result)

        # package and save trace
        print('processing trace data')
        evd_trace = self.pack_evd_trace(self._tf_samples,timeVal)
        evd_trajectory.trace = evd_trace

    def _sample_cb(self, event):
        for group in self._link_groups.keys():
            for link in self._link_groups[group]:
                (pos,rot) = self._listener.lookupTransform(self._fixed_frame, link, rospy.Time(0))
                self._tf_samples[group][link].append(TraceDataPoint(
                    position=Position.from_list(pos),
                    orientation=Orientation.from_list(rot)
                ))

    def start_sample_timer(self, sample_rate=0.1):
        self._timer = rospy.Timer(rospy.Duration(sample_rate),self._sample_cb)

    def RobotStop_sample_timer(self):
        self._timer.shutdown()
        self._timer = None

    def spin(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # check if job
            if len(self._waypoint_jobs) > 0:
                job = self._waypoint_jobs.pop(0)
                self.generate_waypoint(job['data'])
                self._issue_client.clear_pending_job('trace_processor',job['uuid'])

            elif len(self._trace_jobs) > 0:
                job = self._trace_jobs.pop(0)
                self.generate_trace(job['data'])
                self._issue_client.clear_pending_job('trace_processor',job['uuid'])

            rate.sleep()

    def pack_robot_trajectory(self, evd_trajectory):
        moves = []

        if evd_trajectory.move_type != 'planner':

            loc_uuid = evd_trajectory.start_location_uuid
            loc = self._data_client.cache.get(loc_uuid, 'location')
            if evd_trajectory.move_type == 'joint':
                if loc.joints != None and len(loc.joints) > 0:
                    move = self._ursim.pack_robot_move_joint(loc.joints)
                else:
                    move = self._ursim.pack_robot_move_pose_joint(loc.to_ros())
            elif evd_trajectory.move_type == 'linear':
                move = self._ursim.pack_robot_move_pose_linear(loc.to_ros())

            moves.append(move)

            for wp_uuid in evd_trajectory.waypoint_uuids:
                wp = self._data_client.cache.get(wp_uuid, 'waypoint')
                if evd_trajectory.move_type == 'joint':
                    if wp.joints != None and len(wp.joints) > 0:
                        move = self._ursim.pack_robot_move_joint(wp.joints)
                    else:
                        move = self._ursim.pack_robot_move_pose_joint(wp.to_ros())
                elif evd_trajectory.move_type == 'linear':
                    move = self._ursim.pack_robot_move_pose_linear(loc.to_ros())

                moves.append(move)

            loc_uuid = evd_trajectory.end_location_uuid
            loc = self._data_client.cache.get(loc_uuid, 'location')
            if evd_trajectory.move_type == 'joint':
                if loc.joints != None and len(loc.joints) > 0:
                    move = self._ursim.pack_robot_move_joint(loc.joints)
                else:
                    move = self._ursim.pack_robot_move_pose_joint(loc.to_ros())
            elif evd_trajectory.move_type == 'linear':
                move = self._ursim.pack_robot_move_pose_linear(loc.to_ros())

            moves.append(move)

        else: # For planner based trajectories
            loc_uuid = evd_trajectory.start_location_uuid
            startLoc = self._data_client.cache.get(loc_uuid, 'location')

            waypoints = []
            for wp_uuid in evd_trajectory.waypoint_uuids:
                wp = self._data_client.cache.get(wp_uuid, 'waypoint')
                waypoints.append(wp.to_ros())

            loc_uuid = evd_trajectory.end_location_uuid
            endLoc = self._data_client.cache.get(loc_uuid, 'location')

            moves = self.run_planner(startLoc.to_ros(), waypoints, endLoc.to_ros())

        full_traj = RobotMoveTrajectoryGoal()
        full_traj.moves = moves
        full_traj.wait_for_finish = True
        return full_traj

    def pack_evd_trace(self, samples, time):
        return Trace(
            eePath=self._link_groups["end_effector_path"],
            data=samples,
            jPaths=self._link_groups["joint_paths"],
            tPaths=self._link_groups["tool_paths"],
            cPaths=self._link_groups["component_paths"],
            time = time)

    def set_robot_joints(self, joints, run=True):
        move = self._ursim.pack_robot_move_joint(joints)

        trajectory = RobotMoveTrajectoryGoal()
        trajectory.moves = [move]
        trajectory.wait_for_finish = True

        if run:
            result = self.perform_robot_trajectory(trajectory)
        else:
            result = None

        return move, result

    def set_robot_pose(self, pose, run=True):
        move = self._ursim.pack_robot_move_pose_linear(pose)

        trajectory = RobotMoveTrajectoryGoal()
        trajectory.moves = [move]
        trajectory.wait_for_finish = True

        if run:
            result = self.perform_robot_trajectory(trajectory)
        else:
            result = None

        return move, result

    def perform_robot_trajectory(self, trajectory):
        self._ursim.move_trajectory_action.send_goal(trajectory)
        self._ursim.move_trajectory_action.wait_for_result()
        return self._ursim.move_trajectory_action.get_result()

    def run_planner(self, start_pose, waypoint_poses, end_pose):
        #TODO need to integrate a planner into this
        return []


if __name__ == "__main__":
    rospy.init_node('trace_processor')

    node = TraceProcessor()
    node.spin()
