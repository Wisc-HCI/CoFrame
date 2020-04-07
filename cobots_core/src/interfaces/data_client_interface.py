'''
TODO need to think about the write portion more
'''

import json
import rospy
import traceback

from std_msgs.msg import String
from cobots_core.msg import UpdateData, Version

from cobots_core.srv import GetData, GetDataRequest, GetDataResponse
from cobots_core.srv import SetData, SetDataRequest, SetDataResponse
from cobots_core.srv import LoadData, LoadDataRequest, LoadDataResponse
from cobots_core.srv import SaveData, SaveDataRequest, SaveDataResponse

from cobots_model.program.program import *
from cobots_model.environment.environment import *
from cobots_model.version_tracking.history import *


class DataClientInterface(object):

    def __init__(self, use_application_interface=False, use_program_interface=False,
                 use_environment_interface=False, on_program_update_cb=None,
                 on_environment_update_cb=None):

        self._use_application_interface = use_application_interface
        self._use_program_interface = use_program_interface
        self._use_environment_interface = use_environment_interface

        if self._use_application_interface:
            self._load_app_srv = rospy.ServiceProxy('data_server/load_application_data',LoadData)
            self._save_app_srv = rospy.ServiceProxy('data_server/save_application_data',SaveData)
            self._get_app_options_srv = rospy.ServiceProxy('data_server/get_application_options',GetOptions)

        if self._use_program_interface:
            self._program = None
            self._program_verison = None
            self._program_changes_manifest = []
            self._on_program_update_cb = on_program_update_cb

            self._update_program_sub = rospy.Subscriber('data_server/update_program',UpdateData, self._update_program_cb)

            self._get_program_srv = rospy.ServiceProxy('data_server/get_program_data',GetData)
            self._set_program_srv = rospy.ServiceProxy('data_server/set_program_data',SetData)

        if self._use_environment_interface:
            self._envrionment = None
            self._environment_version = None
            self._environment_changes_manifest = []
            self._on_environment_update_cb = on_environment_update_cb

            self._update_env_sub = rospy.Subscriber('data_server/update_environment',UpdateData, self._update_env_cb)

            self._get_env_srv = rospy.ServiceProxy('data_server/get_environment_data',GetData)
            self._set_env_srv = rospy.ServiceProxy('data_server/set_environment_data',SetData)

    '''
    Application Interface
    '''

    def load_application(self, filename):
        if self._use_application_interface:
            response = self._load_data_srv(filename)
            return response.status, response.message
        else:
            raise Exception('Not using application interface')

    def save_application(self, filename=None):
        if self._use_application_interface:
            response = self._save_data_srv(filename == None, filename if filename != None else '')
            return response.status, response.message
        else:
            raise Exception('Not using application interface')

    def get_applications_options(self):
        if self._use_application_interface:
            response = self._get_app_options_srv()
            return response.options
        else:
            raise Exception('Not using application interface')

    '''
    Program Interface
    '''

    @property
    def program(self):
        if self._use_program_interface:
            return self._program
        else:
            raise Exception('Not using program interface')

    @property
    def has_local_program_changes(self):
        if self._use_program_interface:
            return len(self._program_changes_manifest) > 0
        else:
            raise Exception('Not using program interface')

    def get_program_data(self, fetch=False):
        if self._use_program_interface:
            if fetch:
                response = self._get_program_srv(True,'')
                if response.status:
                    self._program = Program.from_dct(json.loads(response.data))
                    self._program_verison = VersionTag.from_ros(response.tag)
                    return self._program
                else:
                    raise Exception(response.message)
            else:
                return self._program
        else:
            raise Exception('Not using program interface')

    def push_program_changes(self):
        if self._use_program_interface:
            # set service with manifest
            # TODO  determine manifest structure
            return None
        else:
            raise Exception('Not using program interface')

    '''
    Environment Interface
    '''

    @property
    def environment(self):
        if self._use_environment_interface:
            return self._environment
        else:
            raise Exception('Not using environment interface')

    @property
    def has_local_environment_changes(self):
        if self._use_environment_interface:
            return len(self._environment_changes_manifest) > 0
        else:
            raise Exception('Not using environment interface')

    def get_environment_data(self, uuids=None, fetch=False):
        if self._use_environment_interface:
            if fetch:
                response = self._get_env_srv(True,'')
                if response.status:
                    self._environment = Environment.from_dct(json.loads(response.data))
                    self._environment_verison = VersionTag.from_ros(response.tag)
                    return self._program
                else:
                    raise Exception(response.message)
            else:
                return self._environment
        else:
            raise Exception('Not using environment interface')

    def push_environment_changes(self):
        if self._use_environment_interface:
            # set service with manifest
            #TODO determine manifest structure
            return None
        else:
            raise Exception('Not using environment interface')

    '''
    Private - Utility
    '''

    def _update_program_cb(self, msg):
        #TODO need to do a smarter version of this where if our tag matches previous tag then only perform the update to current tag
        try:
            self._program = Program.from_dct(json.loads(msg.data))
            self._program.changes_cb = self.__program_changed_cb
            self._program_verison = VersionTag.from_ros(msg.currentTag)
        except:
            traceback.print_exc()
            return #Error parsing, ignore for now

        if self._on_program_update_cb != None:
            self._on_program_update_cb()

    def _update_env_cb(self, msg):
        #TODO need to do a smarter version of this where if our tag matches previous tag then only perform the update to current tag
        try:
            self._environment = Environment.from_dct(json.loads(msg.data))
            self._environment.changes_cb = self.__environment_changed_cb
            self._environment_verison = VersionTag.from_ros(msg.currentTag)
        except:
            traceback.print_exc()
            return #Error parsing, ignore for now

        if self._on_environment_update_cb != None:
            self._on_environment_update_cb()

    def __program_changed_cb(self, trace):
        #TODO need to keep a manifest of all changed nodes
        # self._program_changes_manifest
        print trace

    def __environment_changed_cb(self, trace):
        #TODO need to keep a manifest of all changed nodes
        # self._environment_changes_manifest
        print trace
