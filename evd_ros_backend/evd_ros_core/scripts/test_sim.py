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


class TestSimNode:

    def __init__(self, config_path, config_file_name):
        
        with open(os.path.join(config_path, config_file_name),'r') as f:
            self._config = json.load(f)

        self.ltk = LivelyTKSolver(os.path.join(config_path,'lively-tk',self._config['lively-tk']['config']))
        self.pyb = PyBulletModel(os.path.join(config_path,'pybullet/'), self._config['pybullet'], gui=True)

        self._lively_js_pub = rospy.Publisher('lively_tk/joints',JointState,queue_size=10)
        self._pybullet_js_pub = rospy.Publisher('pybullet/joints',JointState,queue_size=10)

    def spin(self):

        self.ltk.reset()
        self.pyb.reset()

        rate = rospy.Rate(1/self._config['pybullet']['timestep'])
        while not rospy.is_shutdown():
            
            # linear interpolate between poses
            pose = Pose()
            pose.position = Point(0,0,0)
            pose.orientation = Quaternion(0,0,0,1)

            # run model
            lv_joints, lv_names, lv_frames = self.ltk.step(pose)
            #print('Lively Joint Msg:',lv_jMsg)
            pb_joints, pb_names, pb_frames = self.pyb.step(lv_joints, lv_names)
            pb_collisions = self.pyb.collision()
            #print('Pybullet Joint Msg:',pb_jMsg)

            # Publish results
            lv_jMsg = JointState()
            lv_jMsg.name = lv_names
            lv_jMsg.position = lv_joints
            self._lively_js_pub.publish(lv_jMsg)

            pb_jMsg = JointState()
            pb_jMsg.name = pb_names
            pb_jMsg.position = pb_joints
            self._pybullet_js_pub.publish(pb_jMsg)

            rate.sleep()

        self.pyb.cleanup()


if __name__ == "__main__":
    rospy.init_node('test_sim')

    config_path = rospy.get_param('~config_path')
    config_file_name = rospy.get_param("~config_file_name")

    node = TestSimNode(config_path, config_file_name)
    node.spin()