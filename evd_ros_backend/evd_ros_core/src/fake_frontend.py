#!/usr/bin/env python3

'''
Mocked-up the expected interface between the javascript frontend and the ROS system.
'''

import json
import rospy

from std_msgs.msg import String, Empty
from evd_ros_core.msg import Job, Issue, StringArray

from evd_script import Program, NodeParser
from evd_script.examples import CreateDebugApp


class FakeFrontendNode:

    def __init__(self, prefix='', rate=1, default_program=None):
        self._prefix = prefix
        prefix_fmt = prefix + '/' if prefix != '' else prefix
        self._rate = rate

        self._issues = {}
        if default_program == None:
            self._program = Program()
        else:
            self._program = default_program

        self._registration_pub = rospy.Publisher('{0}program/call_to_register'.format(prefix_fmt), Empty, queue_size=5)
        self._registration_sub = rospy.Subscriber('{0}program/register'.format(prefix_fmt), StringArray, self._program_register_cb)
        self._update_pub = rospy.Publisher('{0}program/update'.format(prefix_fmt), String, queue_size=5)

        self._issues_submit_sub = rospy.Subscriber('{0}program/submit/issue'.format(prefix_fmt), Issue, self._issue_submit_cb)
        self._issues_clear_sub = rospy.Subscriber('{0}program/clear/issue'.format(prefix_fmt), Issue, self._issue_clear_cb)

        self._trace_request_pub = rospy.Publisher('{0}program/request/trace'.format(prefix_fmt), Job, queue_size=5)
        self._trace_submit_sub = rospy.Subscriber('{0}program/submit/trace'.format(prefix_fmt), Job, self._trace_submit_cb)
        self._trace_clear_pub = rospy.Publisher('{0}program/clear/trace'.format(prefix_fmt), String, queue_size=5)

        self._joints_request_pub = rospy.Publisher('{0}program/request/joints'.format(prefix_fmt), Job, queue_size=5)
        self._joints_submit_sub = rospy.Subscriber('{0}program/submit/joints'.format(prefix_fmt), Job, self._joints_submit_cb)
        self._joints_clear_pub = rospy.Publisher('{0}program/clear/joints'.format(prefix_fmt), String, queue_size=5)

        self._timer = rospy.Timer(rospy.Duration(0.5), self._update_cb)

    def _program_register_cb(self, msg):
        for entry in msg.data:
            node = NodeParser(json.loads(entry))
            self._program.add_child(node)

    def _trace_submit_cb(self, msg):
        pass

    def _joints_submit_cb(self, msg):
        pass

    def _issue_submit_cb(self, msg):
        self._issues[msg.id] = msg

    def _issue_clear_cb(self, msg):
        if msg.id in self._issues.keys():
            del self._issues[msg.id]
        else:
            pass # trying to delete non-existent issue (ignore)

    def _update_cb(self, event=None):
        data = json.dumps(self._program.to_dct())
        self._update_pub.publish(String(data))

    def spin(self):

        rospy.sleep(2.5) #seconds

        self._registration_pub.publish(Empty()) #request registration

        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            #TODO run frontend CLI
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('fake_frontend')

    prefix = rospy.get_param('~prefix','')
    rate = rospy.get_param('~rate',1)
    load_debug_prog = rospy.get_param('~load_debug_program',False)

    default_prog = CreateDebugApp() if load_debug_prog else None

    node = FakeFrontendNode(prefix,rate,default_prog)
    node.spin()