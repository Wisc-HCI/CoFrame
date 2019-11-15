#!/usr/bin/env python

import rospy
import roslibpy

from std_msgs.msg import Header
from actionlib_msgs.msg import GoalStatusArray, GoalID
from control_msgs.msg import GripperCommandGoal, GripperCommand
from control_msgs.msg import GripperCommandActionGoal, GripperCommandActionFeedback, GripperCommandActionResult


class GripperActionRepublisher:

    def __init__(self, rosbridge_host, rosbridge_port, bridge_name_prefix):
        self._bridge_client = roslibpy.Ros(host=rosbridge_host,port=rosbridge_port)

        not_setup = True
        while not rospy.is_shutdown() and not_setup:
            try:
                self._bridge_client.run()
                not_setup = False
            except:
                print 'Waiting for ROSBridge to connect'
            rospy.sleep(0.25)

        self._ros_goal_pub = rospy.Publisher('gripper_command/goal',GripperCommandActionGoal,queue_size=10)
        self._ros_cancel_pub = rospy.Publisher('gripper_command/cancel',GoalID,queue_size=10)

        self._bridge_status_pub = roslibpy.Topic(self._bridge_client,'{}/gripper_command/status'.format(bridge_name_prefix),'actionlib_msgs/GoalStatusArray')
        self._bridge_result_pub = roslibpy.Topic(self._bridge_client,'{}/gripper_command/result'.format(bridge_name_prefix),'control_msgs/GripperCommandActionResult')
        self._bridge_feedback_pub = roslibpy.Topic(self._bridge_client,'{}/gripper_command/feedback'.format(bridge_name_prefix),'control_msgs/GripperCommandActionFeedback')

        self._ros_status_sub = rospy.Subscriber('gripper_command/status',GoalStatusArray,self._status_ros_cb)
        self._ros_result_sub = rospy.Subscriber('gripper_command/result',GripperCommandActionResult,self._result_ros_cb)
        self._ros_feedback_sub = rospy.Subscriber('gripper_command/feedback',GripperCommandActionFeedback,self._feedback_ros_cb)

        self._bridge_goal_sub = roslibpy.Topic(self._bridge_client,'{}/gripper_command/goal'.format(bridge_name_prefix),'control_msgs/GripperCommandActionGoal')
        self._bridge_goal_sub.subscribe(self._goal_bridge_cb)
        self._bridge_cancel_sub = roslibpy.Topic(self._bridge_client,'{}/gripper_command/cancel'.format(bridge_name_prefix),'actionlib_msgs/GoalID')
        self._bridge_cancel_sub.subscribe(self._cancel_bridge_cb)

    def _goal_bridge_cb(self, dct):
        self._ros_goal_pub.publish(GripperCommandActionGoal(
            header=self.__pack_header(dct['header']),
            goal_id=self.__pack_goal_id(dct['goal_id']),
            goal=GripperCommandGoal(command=GripperCommand(
                position=dct['goal']['command']['position'],
                max_effort=dct['goal']['command']['max_effort']))
        ))

    def _cancel_bridge_cb(self, dct):
        self._ros_cancel_pub.publish(self.__pack_goal_id(dct))

    def _status_ros_cb(self, msg):
        self._bridge_status_pub.publish({
            'header': self.__header_parse(msg.header),
            'status_list': [self.__status_parse(s) for s in msg.status_list]
        })

    def _result_ros_cb(self, msg):
        self._bridge_result_pub.publish({
            'header': self.__header_parse(msg.header),
            'status': self.__status_parse(msg.status),
            'result': {
                'position': msg.result.position,
                'effort': msg.result.effort,
                'stalled': msg.result.stalled,
                'reached_goal': msg.result.reached_goal
            }
        })

    def _feedback_ros_cb(self, msg):
        self._bridge_feedback_pub.publish({
            'header': self.__header_parse(msg.header),
            'status': self.__status_parse(msg.status),
            'feedback': {
                'position': msg.feedback.position,
                'effort': msg.feedback.effort,
                'stalled': msg.feedback.stalled,
                'reached_goal': msg.feedback.reached_goal
            }
        })

    def __status_parse(self, msg):
        return {
            'goal_id': {
                'stamp': msg.stamp,
                'id': msg.id
            },
            'status': msg.status,
            'text': msg.text
        }

    def __header_parse(self, msg):
        return {
            'seq': msg.seq,
            'stamp': msg.stamp,
            'frame_id': msg.frame_id
        }

    def __pack_header(self, dct):
        return Header(
            seq=dct['seq'],
            stamp=dct['stamp'],
            frame_id=dct['frame_id'])

    def __pack_goal_id(self, dct):
        return GoalID(
            stamp=dct['stamp'],
            id=dct['id'])


if __name__ == "__main__":
    rospy.init_node('gripper_action_republisher')

    rosbridge_host = rospy.get_param('~rosbridge_host',None)
    rosbridge_port = rospy.get_param('~rosbridge_port',None)
    bridge_name_prefix = rospy.get_param('~bridge_name_prefix',None)

    node = GripperActionRepublisher(rosbridge_host, rosbridge_port, bridge_name_prefix)
    rospy.spin()
