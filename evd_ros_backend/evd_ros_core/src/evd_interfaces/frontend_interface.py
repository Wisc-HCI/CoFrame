'''
Frontend Interface defines the generalized expectations of the ROSBridged frontend UI.

Since the frontend drives the ROS behavior this node is a shallow interface to handle node
connection, registration, etc.

Please select the specific behavior options when you initialize the interface.
'''

import json
import rospy

from functools import partial
from evd_script import NodeParser

from std_msgs.msg import String, Empty
from evd_ros_core.msg import Job, Issue, StringArray


class FrontendInterface:

    def __init__(self, prefix='', use_update=False, use_registration=False, use_issues=False,
                 update_cb=None, register_cb=None):
        self._prefix = prefix
        prefix_fmt = prefix + '/' if prefix != '' else prefix
        self._should_register = False
        self._job_providers = {}

        self._user_update_cb = update_cb
        self._user_register_cb = register_cb

        if use_registration:
            self._registration_sub = rospy.Subscriber('{0}program/call_to_register'.format(prefix_fmt), Empty, self._register_cb)
            self._registration_pub = rospy.Publisher('{0}program/register'.format(prefix_fmt), StringArray, queue_size=5)
        else: 
            self._registration_sub = None
            self._registration_pub = None
        
        if use_update:
            self._update_sub = rospy.Subscriber('{0}program/update'.format(prefix_fmt), String, self._update_cb)
        else:
            self._update_sub = None

        if use_issues:
            self._issues_submit_pub = rospy.Publisher('{0}program/submit/issue'.format(prefix_fmt), Issue, queue_size=5)
            self._issues_clear_pub = rospy.Publisher('{0}program/clear/issue'.format(prefix_fmt), Issue, queue_size=5)
        else:
            self._issues_submit_pub = None
            self._issues_clear_pub = None

    def _update_cb(self, msg):
        if self._user_update_cb != None:

            program = NodeParser(json.loads(msg.data))
            self._user_update_cb(program)

    def _register_cb(self, _):
        self._should_register = True
        if self._user_register_cb != None:
            self._user_register_cb()
            self._should_register = False

    def _jobs_request_cb(self, job_type, msg):
        self._job_providers[job_type]['user_request_cb'](msg.id, msg.data)

    def _jobs_clear_cb(self, job_type, msg):
        self._job_providers[job_type]['user_clear_cb'](msg.id)

    @property
    def should_register(self):
        if self._registration_pub == None:
            raise Exception("Must define interface with use_registration")
        return self._should_register

    def register(self, dct_list):
        if self._registration_pub == None:
            raise Exception('Must define interface with use_registration')

        self._should_register = False

        msg = StringArray()
        for dct in dct_list:
            msg.data.append(json.dumps(dct))
        self._registration_pub.publish(msg)
    
    def submit_issue(self, source, id, level=None, data=None, description=''):
        if self._issues_submit_pub == None:
            raise Exception('Must define interface with use_issues')

        msg = Issue()
        msg.source = source
        msg.id = id
        msg.level = level if level != None else Issue.LEVEL_ERROR
        msg.data = json.dumps(data) if data != None else '{}'
        msg.description = description
        self._issues_submit_pub.publish(msg)

    def clear_issue(self, source, id):
        if self._issues_clear_pub == None:
            raise Exception('Must define interface with use_issues')
        
        msg = Issue()
        msg.source = source
        msg.id = id
        msg.level = Issue.LEVEL_CLEAR
        self._issues_clear_pub.publish(msg)

    def create_job_provider(self, job_name, request_cb, clear_cb):
        prefix_fmt = self._prefix + '/' if self._prefix != '' else self._prefix

        if job_name in self._job_providers.keys():
            raise Exception('Job with name `{0}` already created'.format(job_name))

        bound_request_cb = partial(self._jobs_request_cb,job_name)
        bound_clear_cb = partial(self._jobs_clear_cb,job_name)

        self._job_providers = {
            'user_request_cb': request_cb,
            'user_clear_cb': clear_cb,
            'request_sub': rospy.Subscriber('{0}program/request/{1}'.format(prefix_fmt,job_name), Job, bound_request_cb),
            'submit_pub': rospy.Publisher('{0}program/submit/{1}'.format(prefix_fmt,job_name), Job, queue_size=5),
            'clear_sub': rospy.Subscriber('{0}program/clear/{1}'.format(prefix_fmt,job_name), String, bound_clear_cb)
        }
    
    def submit_job(self, job_name, id, data):
        self._job_providers[job_name]['submit_pub'].publish(Job(id,data))