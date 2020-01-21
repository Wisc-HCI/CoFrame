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

import tf
import json
import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from cobots_core.msg import Stop, Servo, Move
from cobots_core.msg import MoveTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point

#from rik_interface import RelaxedIKInterface
from robot_interface import RobotInterface


class PlannerServer:

    def __init__(self):
        self._ursim = RobotInterface('planner')
        self._links = [
            'planner_base_link',
            'planner_shoulder_link',
            'planner_upper_arm_link',
            'planner_forearm_link',
            'planner_wrist_1_link',
            'planner_wrist_2_link',
            'planner_wrist_3_link',
            'planner_ee_link',
            'planner_robotiq_85_left_finger_tip_link',
            'planner_robotiq_85_right_finger_tip_link'
            ]
        self._fixed_frame = '/app'

        self._listener = tf.TransformListener()

        self._jobs = {}

    def _generate_trace(self, job_data):

        # unpack trajectory
        print 'unpacking trajectory'
        #TODO currently hard-coded
        trajectory = {
            'waypoints': [
                {
                    'joints': [-1.48055,-1.77753,-1.98455,-0.98455,1.59455,0.08926]
                },
                {
                    'joints': [-3.20208,-1.63108,-2.14362,-0.9582,1.5476,-1.63179]
                }
            ]
        }
        print trajectory

        # reset state of robot to initial joint state
        print 'resetting state of robot'
        start_move = Move()
        start_move.motion_type = Move.JOINT
        start_move.use_ur_ik = False
        start_move.target_joints = trajectory['waypoints'][0]['joints']
        start_move.radius = Move.STD_RADIUS
        start_move.acceleration = Move.JOINT_ACCELERATION
        start_move.velocity = Move.JOINT_VELOCITY

        start_traj = MoveTrajectoryGoal()
        start_traj.moves = [start_move]
        start_traj.wait_for_finish = True
        print start_traj

        self._ursim.move_trajectory_action.send_goal(start_traj)
        self._ursim.move_trajectory_action.wait_for_result()
        result = self._ursim.move_trajectory_action.get_result()
        print result

        # generate trajectory path for robot
        print 'generating full trajectory path'
        moves = []
        for wp in trajectory['waypoints']:
            move = Move()
            move.motion_type = Move.JOINT
            move.use_ur_ik = False
            move.target_joints = wp['joints']
            move.radius = Move.STD_RADIUS
            move.acceleration = Move.JOINT_ACCELERATION
            move.velocity = Move.JOINT_VELOCITY

            moves.append(move)
        full_traj = MoveTrajectoryGoal()
        full_traj.moves = moves
        full_traj.wait_for_finish = True
        print full_traj

        # run trace, collecting links of interest at sample frequency
        print 'running full trajectory and capturing trace data'
        self._samples = []
        self._start_sample_timer()
        self._ursim.move_trajectory_action.send_goal(full_traj)
        self._ursim.move_trajectory_action.wait_for_result()
        result = self._ursim.move_trajectory_action.get_result()
        self._stop_sample_timer()
        print result

        # convert trace into
        # - link-paths
        # - mesh
        print 'processing trace data'
        print self._samples
        link_paths = {}
        point_cloud = PointCloud()
        point_cloud.header.frame_id = self._fixed_frame
        for entry in self._samples:
            for link in entry.keys():
                if not link in link_paths.keys():
                    link_paths[link] = []

                pos = entry[link]['position']
                rot = entry[link]['rotation']

                link_paths[link].append(Pose(
                    position=Vector3(
                        x=pos[0],
                        y=pos[1],
                        z=pos[2]),
                    orientation=Quaternion(
                        x=rot[0],
                        y=rot[1],
                        z=rot[2],
                        w=rot[3])))

    def _sample_cb(self, event):
        data = {}
        for link in self._links:
            (pos,rot) = self._listener.lookupTransform(self._fixed_frame, link, rospy.Time(0))
            data[link] = {
                'position': pos,
                'rotation': rot
            }
        self._samples.append(data)

    def _start_sample_timer(self, sample_rate=0.1):
        self._timer = rospy.Timer(rospy.Duration(sample_rate),self._sample_cb)

    def _stop_sample_timer(self):
        self._timer.shutdown()
        self._timer = None

    def spin(self):

        while not rospy.is_shutdown():

            # check if job
            if len(self._jobs.keys()) > 0:
                self._generate_trace(self._jobs[self._jobs.keys[0]])
            else:
                rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node('planner_server')

    node = PlannerServer()
    node.spin()
