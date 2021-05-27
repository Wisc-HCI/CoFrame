'''
Wraps the interaction with the high-level robot control server and program runner.

This interface reduces the boilerplate to control the robot/program execution.
'''

import json
import rospy

from std_msgs.msg import Empty, Bool, String
from evd_ros_core.msg import ProgramRunnerStatus
from evd_ros_core.srv import SetRootNode, GetRootNode


class RobotControlInterface:

    def __init__(self, at_start_cb=None, at_end_cb=None, lockout_cb=None, tokens_cb=None, status_cb=None, error_cb=None):

        self._user_at_start_cb = at_start_cb
        self._user_at_end_cb = at_end_cb
        self._user_lockout_cb = lockout_cb
        self._user_tokens_cb = tokens_cb
        self._user_status_cb = status_cb
        self._user_error_cb = error_cb

        self.set_root_node_srv = rospy.ServiceProxy('robot_control_server/set_root_node',SetRootNode)
        self.get_root_node_srv = rospy.ServiceProxy('robot_control_server/get_root_node',GetRootNode)

        self.play_pub = rospy.Publisher('robot_control_server/play',Empty,queue_size=10)
        self.stop_pub = rospy.Publisher('robot_control_server/stop',Empty,queue_size=10)
        self.pause_pub = rospy.Publisher('robot_control_server/pause',Empty,queue_size=10)
        self.reset_pub = rospy.Publisher('robot_control_server/reset',Empty,queue_size=10)

        self.at_start_sub = rospy.Subscriber('robot_control_server/at_start',Bool,self._at_start_cb)
        self.at_end_sub = rospy.Subscriber('robot_control_server/at_end',Bool,self._at_end_cb)
        self.lockout_sub = rospy.Subscriber('robot_control_server/lockout',Bool,self._lockout_cb)
        self.status_sub = rospy.Subscriber('robot_control_server/status',ProgramRunnerStatus,self._status_cb)
        self.tokens_sub = rospy.Subscriber('robot_control_server/tokens',String,self._tokens_cb)
        self.errors_sub = rospy.Subscriber('robot_control_server/error',String,self._error_cb)

    def _at_start_cb(self, msg):
        if self._user_at_start_cb != None:
            self._user_at_start_cb(msg.data)

    def _at_end_cb(self, msg):
        if self._user_at_end_cb != None:
            self._user_at_end_cb(msg.data)

    def _lockout_cb(self, msg):
        if self._user_lockout_cb != None:
            self._user_lockout_cb(msg.data)

    def _status_cb(self, msg):
        if self._user_status_cb != None:
            self._user_status_cb(msg)

    def _tokens_cb(self, msg):
        if self._user_tokens_cb != None:
            self._user_tokens_cb(json.loads(msg.data))

    def _error_cb(self, msg):
        if self._user_error_cb != None:
            self._user_error_cb(msg.data)

    def set_root_node(self, uuid):
        response = self.set_root_node_srv(uuid)
        return response.status, response.message

    def get_root_node(self):
        return self.get_root_node_srv().uuid

    def play(self):
        self.play_pub.publish(Empty())

    def stop(self):
        self.stop_pub.publish(Empty())

    def pause(self):
        self.pause_pub.publish(Empty())

    def reset(self):
        self.reset_pub.publish(Empty())
