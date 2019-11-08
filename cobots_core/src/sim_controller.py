#!/usr/bin/env python

import rospy


class FakeController:
    pass



import rospy
import actionlib

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandFeedback, GripperCommandResult


class FollowJointTrajectoryServer:

    def __init__(self):
        self._virtualController = rospy.Publisher('virtual_controller/joint_states',JointState,queue_size=5)
        self._server = actionlib.SimpleActionServer('follow_joint_trajectory', FollowJointTrajectoryAction, self._execute, False)
        self._server.start()

    def _execute(self, goal):
        status = 0

        time = 0
        for i in range(0,len(goal.trajectory.points)):

            if self._server.is_preempt_requested():
                status = -3
                break

            jointState = JointState()
            jointState.name = goal.trajectory.joint_names
            jointState.position = goal.trajectory.points[i].positions
            jointState.velocity = goal.trajectory.points[i].velocities
            self._virtualController.publish(jointState)

            delay = goal.trajectory.points[i].time_from_start.to_sec() - time
            rospy.sleep(delay)
            time += delay

            num_joints = len(goal.trajectory.joint_names)

            feedback = FollowJointTrajectoryFeedback()
            feedback.joint_names = goal.trajectory.joint_names
            feedback.desired = goal.trajectory.points[i]
            feedback.actual = goal.trajectory.points[i]
            feedback.error = JointTrajectoryPoint(positions=[0]*num_joints,
                                                  velocities=[0]*num_joints,
                                                  accelerations=[0]*num_joints,
                                                  effort=[0]*num_joints)
            self._server.publish_feedback(feedback)

        result = FollowJointTrajectoryResult()
        result.error_code = status
        if status == -3:
            self._server.set_preempted(result)
        else:
            self._server.set_succeeded(result)


class GripperCommandServer:

    def __init__(self, gripperJoint):
        self._virtualController = rospy.Publisher('virtual_controller/joint_states',JointState,queue_size=5)
        self._server = actionlib.SimpleActionServer('gripper_command', GripperCommandAction, self._execute, False)
        self._server.start()
        self._joint = gripperJoint

    def _execute(self, goal):

        jointState = JointState()
        jointState.name = self._joint if isinstance(self._joint,(list,tuple)) else [self._joint]
        jointState.position = [goal.command.position]*len(jointState.name)
        self._virtualController.publish(jointState)

        result = GripperCommandResult()
        result.position = goal.command.position
        result.stalled = False
        result.reached_goal = True
        self._server.set_succeeded(result)


class JointStateServoingServer:

    def __init__(self, joint_state_topic, time):
        self._time = time
        self._virtualController = rospy.Publisher('virtual_controller/joint_states',JointState,queue_size=5)
        self._servo_cub = rospy.Subscriber(joint_state_topic,JointState,self._servo_cb)

    def _servo_cb(self, msg):
        rospy.sleep(self._time)
        self._virtualController.publish(msg)


if __name__ == "__main__":
    rospy.init_node('virtual_robot_controller')

    fjtServer = FollowJointTrajectoryServer()

    gripperJoint = rospy.get_param("~gripper_joint",None)
    if not gripperJoint is None:
        gcServer = GripperCommandServer(gripperJoint)

    servoing_time = rospy.param("~servoing_time",0.1)
    joint_state_topic = rospy.get_param("~joint_state_topic",'servoing')
    servoingServer = JointStateServoingServer(joint_state_topic,servoing_time)

    while not rospy.is_shutdown():
        rospy.spin()
