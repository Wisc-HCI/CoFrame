#!/usr/bin/env python3

'''
'''

import os
import json
import rospy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion

from evd_sim.lively_tk_solver import LivelyTKSolver
from evd_sim.pybullet_model import PyBulletModel
from evd_sim.pose_reached import poseReached
from evd_sim.pose_interpolator import PoseInterpolator
from evd_sim.joints_stabilized import JointsStabilizedFilter
from evd_sim.joint_interpolator import JointInterpolator


class TestSimNode:

    def __init__(self, config_path, config_file_name):
        
        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)

        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet/'), self._config['pybullet'], gui=True)
        self.jsf = JointsStabilizedFilter(20,0.001)

        self._lively_js_pub = rospy.Publisher('lively_tk/joints',JointState,queue_size=10)
        self._pybullet_js_pub = rospy.Publisher('pybullet/joints',JointState,queue_size=10)

    def spin(self):

        defaultJs, names = self.ltk.reset()
        self.pyb.set_joints(defaultJs, names)

        velocity = 0.05

        pose1 = Pose()
        pose1.position = Point(0,0,0)
        pose1.orientation = Quaternion(0,0,0,1)

        pose2 = Pose()
        pose2.position = Point(0.3,0,0.3)
        pose2.orientation = Quaternion(0,0,0,1)

        state = '1'
        poseInterpolator = PoseInterpolator(pose1, pose2, velocity)
        targetPose = pose2
        time = 0

        rate = rospy.Rate(1/self._config['pybullet']['timestep'])
        while not rospy.is_shutdown():

            # linear interpolate between poses
            pose = poseInterpolator.step(time)

            # run model
            lv_joints, lv_frames = self.ltk.step(pose)
            lv_ee_pose = LivelyTKSolver.get_ee_pose(lv_frames[0])
            self.jsf.append(lv_joints[0])

            pb_joints, pb_frames = self.pyb.step(lv_joints[0], lv_joints[1])
            pb_collisions = self.pyb.collisionCheck()

            # Publish results
            lv_jMsg = JointState()
            lv_jMsg.name = lv_joints[1]
            lv_jMsg.position = lv_joints[0]
            self._lively_js_pub.publish(lv_jMsg)

            pb_jMsg = JointState()
            pb_jMsg.name = pb_joints[2]
            pb_jMsg.position = pb_joints[0]
            pb_jMsg.velocity = pb_joints[1]
            self._pybullet_js_pub.publish(pb_jMsg)

            rate.sleep()

            # swap targets when reached
            eePose = LivelyTKSolver.get_ee_pose(lv_frames[0])
            #print('EE Pose:',eePose,'\nTarget Pose:', targetPose)
            print('Is Stable', self.jsf.isStable(),'Pose Reached:',poseReached(targetPose, eePose, 0.05, 0.02))
            if self.jsf.isStable() and poseReached(targetPose, eePose, 0.05, 0.02):
                #print('\n\n\n\nSWAP\n\n\n\n')
                
                time = 0
                if state == '1':
                    poseInterpolator = PoseInterpolator(pose2, pose1, velocity)
                    targetPose = pose1
                    state = '2'
                else:
                    poseInterpolator = PoseInterpolator(pose1, pose2, velocity)
                    targetPose = pose2
                    state = '1'
            else:
                time += self._config['pybullet']['timestep']

        self.pyb.cleanup()


if __name__ == "__main__":
    rospy.init_node('test_sim')

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")

    node = TestSimNode(config_path, config_file_name)
    node.spin()