#!/usr/bin/env python3

import os
import time
import rospy
import json
import pybullet

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from lively_tk import ObjectiveInput, Solver, parse_config_data
from datetime import datetime



CONFIG_MATCH_FIELDS = {'ee_fixed_joints', 'static_environment','fixed_frame','joint_names', 'joint_ordering'}


class Time(object):
    def __init__(self,sec,nanosec):
        self.sec = sec
        self.nanosec = nanosec


class SolverNode(object):

    def __init__(self, config_file, desc_path):

        self.kp = 200
        self.kd = 1
        self.maxForce = 1000

        #pybullet setup
        physicsClient = pybullet.connect(pybullet.GUI) # or pybullet.DIRECT for non-gui
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_Y_AXIS_UP,1)
        #pybullet.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
        pybullet.setAdditionalSearchPath(desc_path)
        pybullet.setGravity(0,-9.8,0)

        self.timeStep=1./60.
        p.setTimeStep(self.timeStep)

        self.robotId = pybullet.loadURDF('simulated_ur3e_robotiq85_plus_workcell.urdf', [0,0,0], useFixedBase=True)
        self.jointIds = {}
        for j in range(pybullet.getNumJoints(self.robotId)):
            info = pybullet.getJointInfo(self.robotId, j)
            jointName = info[1].decode("utf-8") 
            self.jointIds[jointName] = j

        print(self.jointIds)

        pybullet.setRealTimeSimulation(1)

        # Load config file
        with open(config_file) as f:
            self.config_data = json.load(f)

        self.config = parse_config_data(self.config_data)
        self.solver = Solver(self.config)

        self.iterations = 15*(3+len(self.config_data['starting_config'][1]))
        self.base_transform = self.config_data['starting_config'][0]
        self.displayed_state = self.config_data['starting_config'][1]
        self.current_weights = self.config.default_weights

        # Parse the rust goal specs into a python dictionary
        _directions = []
        for default_goal in self.config.default_goals:
            goal_data = {}
            for field in ['scalar','vector','quaternion']:
                value = getattr(default_goal,field)
                if value:
                    goal_data[field] = value
            _directions.append(goal_data)

        self.target_directions = _directions

        # Define ROS interface
        self._target_pose_sub = rospy.Subscriber('lively_tk/target_pose',Pose,self._target_pose_cb)
        self._result_joints_pub = rospy.Publisher('lively_tk/result_joints',JointState,queue_size=10)

    def _target_pose_cb(self, msg):
        pos = msg.position
        rot = msg.orientation
        
        self.target_directions[0] = {'vector':[pos.x,pos.y,pos.z]}
        self.target_directions[1] = {'quaternion':[rot.w,rot.x,rot.y,rot.z]}

    def spin(self):
        while not rospy.is_shutdown():
            if self.solver != None:
                inputs = []
                for idx, objective in enumerate(self.config.objectives):
                    input_value = {}
                    input_value.update({'weight':self.current_weights[idx]})
                    input_value.update(self.target_directions[idx])
                    inputs.append(ObjectiveInput(**input_value))

                base_tf, joints, _ = self.solver.solve(
                    inputs,
                    datetime.utcnow().timestamp(),
                    max_retries=0,
                    max_iterations=self.iterations)

                msg = JointState()
                msg.name = ['simulated_' + x for x in self.config_data["joint_ordering"]]
                msg.position = joints
                self._result_joints_pub.publish(msg)

                lastTime = time.time()
                # Set each joint in pybullet
                for name, id in self.jointIds.items():

                    index = -1
                    for i, n in enumerate(self.config_data["joint_ordering"]):
                        #print(name, 'simulated_' + self.config_data["joint_ordering"][i])
                        if name == 'simulated_' + n:
                            index = i

                    if index != -1:
                        val = joints[i]
                        pybullet.setJointMotorControl2(self.robotId,id,pybullet.POSITION_CONTROL, targetPosition=val, positionGain=self.kp, force=self.maxForce)

            pybullet.stepSimulation()
            time.sleep(self.timeStep)


if __name__ == "__main__":
    rospy.init_node('lively_demo')

    config_file = rospy.get_param('~config_file')
    desc_path = rospy.get_param('~path')

    node = SolverNode(config_file, desc_path)
    node.spin()