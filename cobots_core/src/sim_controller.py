#!/usr/bin/env python

import tf
import math
import rospy
import actionlib

from ur_kin_py.kin import Kinematics

from std_msgs.msg import Bool
from pyquaternion import Quaternion
from sensor_msgs.msg import JointState
from cobots_core.msg import Stop, Servo, Move
from cobots_core.msg import MoveTrajectoryAction, MoveTrajectoryGoal, MoveTrajectoryResult, MoveTrajectoryFeedback
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandFeedback, GripperCommandResult


MIN_M = -30.0
MAX_M = 1.0
MIN_TIME = 0.05
MAX_TIME = 0.9


class FakeController:

    def __init__(self, ee_frame, base_frame, gain, lookahead, time_scalars, timestep, gripper_joint):
        self._gain = gain
        self._ee_frame = ee_frame
        self._timestep = timestep
        self._joint = gripper_joint
        self._lookahead = lookahead
        self._base_frame = base_frame
        self._running_trajectory = False
        self._time_scalars = time_scalars

        self._tf_listener = tf.TransformListener()

        self._js_pub = rospy.Publisher('sim_controller/joint_state',JointState,queue_size=10)

        self._freedrive_sub = rospy.Subscriber('robot_control/freedrive',Bool,self._freedrive_cb)
        self._servoing_sub = rospy.Subscriber('robot_control/servoing',Servo,self._servoing_cb)
        self._stop_sub = rospy.Subscriber('robot_control/stop',Stop,self._stop_cb)

        self._move_trajectory_as = actionlib.SimpleActionServer('robot_control/move_trajectory',MoveTrajectoryAction,execute_cb=self._move_trajectory_cb,auto_start=False)
        self._move_trajectory_as.start()

        self._gripper_as = actionlib.SimpleActionServer('gripper_command', GripperCommandAction, self._gripper_cb, False)
        self._gripper_as.start()

    def _freedrive_cb(self, msg):
        pass # Not used in simulated controller

    def _servoing_cb(self, msg):
        self._running_trajectory = False

        if msg.motion_type == Servo.CIRCULAR:
            pass # TODO need to figure out how UR does it
        elif msg.motion_type == Servo.JOINT:
            if msg.use_ur_ik:
                pass # TODO need to figure out how UR does it
            else:
                pass #TODO need to figure out how UR does it

    def _move_trajectory_cb(self, goal):
        self._running_trajectory = True

        result = MoveTrajectoryResult()
        result.status = True
        result.message = ''

        feedback = MoveTrajectoryFeedback()
        feedback.message = 'starting action'
        self._move_trajectory_as.publish_feedback(feedback)

        # TODO process path
        for move in goal.moves:
            if move.motion_type == Move.CIRCULAR:
                pass
            elif move.motion_type == Move.JOINT:
                if move.use_ur_ik:
                    pass
                else:
                    pass
            elif move.motion_type == Move.LINEAR:
                pass
            elif move.motion_type == Move.PROCESS:
                pass

        # determine end condition
        if self._running_trajectory:
            self._running_trajectory = False
            feedback = MoveTrajectoryFeedback()
            feedback.message = 'in steady state'
            self._move_trajectory_as.publish_feedback(feedback)
        else:
            result.message = 'extern behavior prematurely stopped action'
            result.status = False

        self._move_trajectory_as.set_succeeded(result)

    def _stop_cb(self, msg):
        self._running_trajectory = False

    def _gripper_cb(self, goal):

        jointState = JointState()
        jointState.name = self._joint if isinstance(self._joint,(list,tuple)) else [self._joint]
        jointState.position = [goal.command.position]*len(jointState.name)
        self._js_pub.publish(jointState)

        result = GripperCommandResult()
        result.position = goal.command.position
        result.stalled = False
        result.reached_goal = True
        self._gripper_as.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node('ur_controller')

    print '\n\n', Kinematics('ur5').forward([0.1]*6), '\n\n'

    gain = rospy.get_param('~gain')
    ee_frame = rospy.get_param('~ee_frame')
    lookahead = rospy.get_param('~lookahead')
    base_frame = rospy.get_param('~base_frame')
    gripper_joint = rospy.get_param("~gripper_joint")
    timestep = rospy.get_param('~timestep',MIN_TIME)
    time_scalars = rospy.get_param('~time_scalars',None)

    node = FakeController(ee_frame,base_frame,gain,lookahead,time_scalars,timestep,gripper_joint)
    rospy.spin()
