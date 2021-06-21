'''
Frontend Interface defines the generalized expectations of the ROSBridged frontend UI.

Since the frontend drives the ROS behavior this node is a shallow interface to handle node
connection, registration, etc.

Please select the specific behavior options when you initialize the interface.
'''

import json
import rospy

from std_msgs.msg import String, Empty
from evd_ros_core.msg import Job, Issue, StringArray


class FrontendInterface:

    def __init__(self, prefix='', use_update=False, use_registration=False, use_trace=False, use_joints=False,
                 use_issues=False, update_cb=None, trace_cb=None, joints_cb=None, register_cb=None):
        self._prefix = prefix
        prefix_fmt = prefix + '/' if prefix != '' else prefix
        self._should_register = False

        self._user_update_cb = update_cb
        self._user_trace_cb = trace_cb
        self._user_joints_cb = joints_cb
        self._user_register_cb = register_cb

        if use_registration:
            self._registration_sub = rospy.Subscriber('{0}program/call_to_register'.format(prefix_fmt), Empty, self._register_cb)
            self._registration_pub = rospy.Publisher('{0}program/register'.format(prefix_fmt, StringArray, queue_size=5))
        else: 
            self._registration_sub = None
            self._registration_pub = None
        
        if use_update:
            self._update_sub = rospy.Subscriber('{0}program/update'.format(prefix_fmt), String, self._update_cb)
        else:
            self._update_sub = None

        if use_trace:
            self._trace_request_sub = rospy.Subscriber('{0}program/request/trace'.format(prefix_fmt), Job, self._trace_request_cb)
            self._trace_submit_pub = rospy.Publisher('{0}program/submit/trace'.format(prefix_fmt), Job, queue_size=5)
            self._trace_clear_sub = rospy.Subscriber('{0}program/clear/trace'.format(prefix_fmt), Job, self._trace_clear_cb)
        else:
            self._trace_request_sub = None
            self._trace_submit_pub = None
            self._trace_clear_sub = None

        if use_joints:
            self._joints_request_sub = rospy.Subscriber('{0}program/request/joints'.format(prefix_fmt), Job, self._joints_request_cb)
            self._joints_submit_pub = rospy.Publisher('{0}program/submit/joints'.format(prefix_fmt), Job, queue_size=5)
            self._joints_clear_sub = rospy.Subscriber('{0}program/clear/joints'.format(prefix_fmt), Job, self._joints_clear_cb)
        else:
            self._joints_request_sub = None
            self._joints_submit_pub = None
            self._joints_clear_sub = None

        if use_issues:
            self._issues_submit_pub = rospy.Publisher('{0}program/submit/issue'.format(prefix_fmt), Issue, queue_size=5)
            self._issues_clear_pub = rospy.Publisher('{0}program/clear/issue'.format(prefix_fmt), Issue, queue_size=5)
        else:
            self._issues_submit_pub = None
            self._issues_clear_pub = None


    def _update_cb(self, msg):
        if self._user_update_cb != None:
            self._user_update_cb(json.loads(msg.data))

    def _trace_request_cb(self, msg):
        if self._user_trace_cb != None:
            self._user_trace_cb(msg.id, 'request', json.loads(msg.data))

    def _trace_clear_cb(self, msg):
        if self._user_trace_cb != None:
            self._user_trace_cb(msg.id, 'clear', None)

    def _joints_request_cb(self, msg):
        if self._user_joints_cb != None:
            self._user_joints_cb(msg.id, 'request', json.loads(msg.data))

    def _joints_clear_cb(self, msg):
        if self._user_joints_cb != None:
            self._user_joints_cb(msg.id, 'clear', None)

    def _register_cb(self, _):
        self._should_register = True
        if self._user_register_cb != None:
            self._user_register_cb()

    @property
    def should_register(self):
        if self._registration_pub == None:
            raise Exception("Must define interface with use_registration")
        return self._should_register

    def register(self, dct):
        if self._registration_pub == None:
            raise Exception('Must define interface with use_registration')

        self._should_register = False

        msg = String(json.dumps(dct))
        self._registration_pub.publish(msg)
    
    def submit_trace(self, dct, id):
        if self._trace_submit_pub == None:
            raise Exception('Must define interface with use_trace')

        msg = Job()
        msg.id = id
        msg.data = json.dumps(dct)
        self._trace_submit_pub.publish(msg)

    def submit_joints(self, dct, id):
        if self._joints_submit_pub != None:
            raise Exception('Must define interface with use_joints')

        msg = Job()
        msg.id = id
        msg.data = json.dumps(dct)
        self._joints_submit_pub.publish(msg)

    def submit_issue(self, source, id, level=None, data=None, description=''):
        if self._issues_submit_pub == None:
            raise Exception('Must define interface with use_issues')

        msg = Issue()
        msg.source = source
        msg.id = id
        msg.level = level if level != None else Issue.LEVEL_ERROR
        msg.data = json.dumps(data) if data != None else '{}'
        msg.human_message = description
        self._issues_submit_pub.publish(msg)

    def clear_issue(self, source, id):
        if self._issues_clear_pub == None:
            raise Exception('Must define interface with use_issues')
        
        msg = Issue()
        msg.source = source
        msg.id = id
        msg.level = Issue.LEVEL_CLEAR
        self._issues_clear_pub.publish(msg)
