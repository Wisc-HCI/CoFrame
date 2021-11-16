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
from evd_ros_core.srv import QueryJob, QueryJobRequest, QueryJobResponse


class FrontendInterface:

    def __init__(self, prefix='', use_update=False, use_registration=False,
                 update_cb=None, register_cb=None, use_processor_configure=False, 
                 processor_configure_cb=None, use_machine_configure=False,
                 machine_configure_cb=None):
        self._prefix = prefix
        prefix_fmt = prefix + '/' if prefix != '' else prefix
        self._should_register = False
        self._job_providers = {}

        self._user_update_cb = update_cb
        self._user_register_cb = register_cb
        self._user_processor_config_cb = processor_configure_cb
        self._user_machine_config_cb = machine_configure_cb

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

        if use_processor_configure:
            self._configure_processors = rospy.Subscriber('{0}program/configure/processors'.format(prefix_fmt), String, self._processor_configure_cb)

        if use_machine_configure:
            self._configure_machines = rospy.Subscriber('{0}program/configure/machines'.format(prefix_fmt), String, self._machine_configure_cb)

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
        self._job_providers[job_type]['user_request_cb'](msg.id, json.loads(msg.data))

    def _jobs_clear_cb(self, job_type, msg):
        self._job_providers[job_type]['user_clear_cb'](msg.id)

    def _jobs_query_cb(self, job_type, _):
        (active_job, pending_jobs) = self._job_providers[job_type]['user_query_cb']()
        response = QueryJobResponse()
        response.active_job - active_job
        response.pending_jobs = pending_jobs
        return response

    def _processor_configure_cb(self, msg):
        if self._user_processor_config_cb != None:
            dct = json.loads(msg.data)

            data = {}
            for key in dct.keys():
                data[key] = []

                for n in dct[key]:
                    data[key].append(NodeParser(n))

            self._user_processor_config_cb(data)

    def _machine_configure_cb(self, msg):
        if self._user_machine_config_cb != None:
            dct = json.loads(msg.data)

            data = {}
            for key in dct.keys():
                data[key] = []

                for n in dct[key]:
                    data[key].append(NodeParser(n))

            self._user_machine_config_cb(data)

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

    def create_job_provider(self, job_name, request_cb, clear_cb, query_cb):
        prefix_fmt = self._prefix + '/' if self._prefix != '' else self._prefix

        if job_name in self._job_providers.keys():
            raise Exception('Job with name `{0}` already created'.format(job_name))

        bound_request_cb = partial(self._jobs_request_cb,job_name)
        bound_clear_cb = partial(self._jobs_clear_cb,job_name)
        bound_query_cb = partial(self._jobs_query_cb,job_name)

        self._job_providers[job_name] = {
            'user_request_cb': request_cb,
            'user_clear_cb': clear_cb,
            'user_query_cb': query_cb,
            'request_sub': rospy.Subscriber('{0}program/request/{1}'.format(prefix_fmt,job_name), Job, bound_request_cb),
            'submit_pub': rospy.Publisher('{0}program/submit/{1}'.format(prefix_fmt,job_name), Job, queue_size=5),
            'clear_sub': rospy.Subscriber('{0}program/clear/{1}'.format(prefix_fmt,job_name), String, bound_clear_cb),
            'query_srv': rospy.Service('{0}program/query/{1}'.format(prefix_fmt,job_name), QueryJob, bound_query_cb)
        }
    
    def submit_job(self, job_name, id, data):
        self._job_providers[job_name]['submit_pub'].publish(Job(id,data))