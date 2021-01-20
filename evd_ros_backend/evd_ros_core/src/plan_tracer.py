#!/usr/bin/env python

'''
Planner server generates traces from trajectories using
- RelaxedIK

Planner traces trajectories using
- URSim

Planner creates full visualizations for trajectories
- end-effector-path
- occupancy volume mesh
- joint-link-paths (more general form of end-effector path)
'''

#TODO generalize this

import tf
import json
import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from evd_ros_core.msg import Stop, Servo, Move
from evd_ros_core.msg import MoveTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point
from evd_ros_core.srv import SubmitJob, SubmitJobRequest, SubmitJobResponse
from evd_ros_core.srv import PendingJobs, PendingJobsRequest, PendingJobsResponse

from interfaces.rik_interface import RelaxedIKInterface
from interfaces.robot_interface import RobotInterface
from interfaces.data_client_interface import DataClientInterface

from evd_script.data.trace import *
from evd_script.data.trajectory import *
from evd_script.data.geometry import *


class PlanTracer:

    def __init__(self):
        self._jobs = []
        self._samples = {}
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

        self._ursim = RobotInterface('planner')
        self._data_client = DataClientInterface(use_program_interface=True)

        self._listener = tf.TransformListener()
        self._submit_job_srv = rospy.Service('plan_tracer/submit_job',SubmitJob,self._submit_job_cb)
        self._pending_jobs_srv = rospy.Service('plan_tracer/pending_jobs',PendingJobs,self._pending_jobs_cb)

    def _submit_job_cb(self, request):
        id = '{}-py-{}'.format('plan_tracer_job',uuid.uuid1().hex)

        self._jobs.append({
            'type': request.type,
            'params': json.loads(request.params),
            'id': id
        })

        response = JobResponse()
        response.job_id = id
        return response

    def _pending_jobs_cb(self, request):
        response = PendingJobsResponse()

        for j in self._jobs:
            response.job_ids.append(j['job_id'])
            response.types.append(j['type'])
            response.params.append(json.dumps(j['params']))

        return response

    def generate_waypoint(self, job_data):
        pass

    def generate_trace(self, job_data):

        # generate trajectory path for robot
        print 'generating full trajectory path'
        fullTraj = self.pack_robot_trajectory(job_data['trajectory_uuid'])

        # reset state of robot to initial joint state
        print 'resetting state of robot'
        # TODO

        # run trace, collecting links of interest at sample frequency
        print 'running full trajectory and capturing trace data'
        self._samples = {gorup:[] for group in self._link_groups.keys()}

        self.start_sample_timer()
        start_time = time.time()
        result = self.perform_robot_trajectory(fullTraj)
        end_time = time.time()
        self.stop_sample_timer()

        timeVal = end_time - start_time
        print result

        # package and save trace
        print 'processing trace data'
        trace = self.pack_trace(self._samples,timeVal)
        #TODO save

    def _sample_cb(self, event):
        for group in self._link_groups.keys():
            for link in self._link_groups[group]:
                (pos,rot) = self._listener.lookupTransform(self._fixed_frame, link, rospy.Time(0))
                self._samples[group][link].append(TraceDataPoint(
                    position=Position.from_list(pos),
                    orientation=Orientation.from_list(rot)
                ))

    def start_sample_timer(self, sample_rate=0.1):
        self._timer = rospy.Timer(rospy.Duration(sample_rate),self._sample_cb)

    def stop_sample_timer(self):
        self._timer.shutdown()
        self._timer = None

    def spin(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # check if job
            if len(self._jobs) > 0:
                job = self._jobs.pop(0)
                if job.type == "trace":
                    self.generate_trace(job)
                elif job.type == "waypoint":
                    self.generate_waypoint(job)
                else:
                    rospy.log('Invalid type provided {}'.format(job.type))

            rate.sleep()

    def pack_robot_trajectory(self, trajId):
        #TODO
        moves = []
        for wp in trajectory.waypoints:
            move = self._ursim.pack_robot_move_joint(wp['joints'])
            moves.append(move)

        full_traj = MoveTrajectoryGoal()
        full_traj.moves = moves
        full_traj.wait_for_finish = True
        return full_traj

    def pack_trace(self, samples, time):
        return Trace(
            eePath=self._link_groups["end_effector_path"],
            data=samples,
            jPaths=self._link_groups["joint_paths"],
            tPaths=self._link_groups["tool_paths"],
            cPaths=self._link_groups["component_paths"],
            time = time
        )

    def set_robot_joints(self, joints, run=True):
        move = self._ursim.pack_robot_move_joint(joints)

        trajectory = MoveTrajectoryGoal()
        trajectory.moves = [move]
        trajectory.wait_for_finish = True

        if run:
            result = self.perform_robot_trajectory(trajectory)
        else:
            result = None

        return move, result

    def set_robot_pose(self, pose, run=True):
        move = self._ursim.pack_robot_move_pose_linear(pose)

        trajectory = MoveTrajectoryGoal()
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


if __name__ == "__main__":
    rospy.init_node('plan_tracer')

    node = PlanTracer()
    node.spin()
