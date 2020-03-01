'''

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
from cobots_core.srv import CreateData, CreateDataRequest, CreateDataResponse
from cobots_core.srv import DeleteData, DeleteDataRequest, DeleteDataResponse

from version_tracking.history import *
from cobots_model.program.program import *

#TODO need to reflect local state of program and transfer state to server

class DataClientInterface(object):

    def __init__(self, use_application_interface=False, use_program_interface=False,
                 use_environment_interface=False, on_program_update_cb=None,
                 on_environment_update_cb=None):
        self._program = None
        self._program_verison = None
        self._program_changes_manifest = []
        self._envrionment = None
        self._environment_version = None
        self._environment_changes_manifest = []

        self._on_program_update_cb = on_program_update_cb
        self._on_environment_update_cb = on_environment_update_cb

        self._use_application_interface = use_application_interface
        self._use_program_interface = use_program_interface
        self._use_environment_interface = use_environment_interface

        if use_application_interface:
            self._load_data_srv = rospy.ServiceProxy('data_server/load_application_data',LoadData)
            self._save_data_srv = rospy.ServiceProxy('data_server/save_application_data',SaveData)
            self._get_app_options_srv = rospy.ServiceProxy('data_server/get_application_options',GetOptions)

        if use_program_interface:
            self._update_program_sub = rospy.Subscriber('data_server/update_program',UpdateData, self._update_program_cb)

            self._get_program_data_srv = rospy.ServiceProxy('data_server/get_program_data',GetData)
            self._set_program_data_srv = rospy.ServiceProxy('data_server/set_program_data',SetData)
            self._create_program_data_srv = rospy.ServiceProxy('data_server/create_program_data',CreateData)
            self._delete_program_data_srv = rospy.ServiceProxy('data_server/delete_program_data',DeleteData)

        if use_environment_interface:
            self._update_env_sub = rospy.Subscriber('data_server/update_environment',UpdateData, self._update_env_cb)

            self._get_env_data_srv = rospy.ServiceProxy('data_server/get_environment_data',GetData)
            self._set_env_data_srv = rospy.ServiceProxy('data_server/set_environment_data',SetData)
            self._create_env_data_srv = rospy.ServiceProxy('data_server/create_environmment_data',CreateData)
            self._delete_env_data_srv = rospy.ServiceProxy('data_server/delete_environment_data',DeleteData)

    @property
    def program(self):
        return self._program

    @property
    def local_program_changes(self):
        return len(self._program_changes_manifest) > 0

    @property
    def environment(self):
        return self._environment

    @property
    def local_environment_changes(self):
        return len(self._environment_changes_manifest) > 0

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
        #TODO parse data into model copy

        if self._on_environment_update_cb != None:
            self._on_environment_update_cb(0)

    def get_program_data(self, fetch=False):
        if self._use_program_interface:
            return None
        else:
            raise Exception('Not using program interface')

    def set_program_data(self, data):
        if self._use_program_interface:
            return None
        else:
            raise Exception('Not using program interface')

    def create_program_data(self, data):
        if self._use_program_interface:
            return None
        else:
            raise Exception('Not using program interface')

    def delete_program_data(self, uuids):
        if self._use_program_interface:
            return None
        else:
            raise Exception('Not using program interface')

    def push_program_manifest(self):
        if self._use_program_interface:
            return None
        else:
            raise Exception('Not using program interface')

    def get_environment_data(self, uuids=None, fetch=False):
        if self._use_environment_interface:
            return None
        else:
            raise Exception('Not using environment interface')

    def set_environment_data(self, data):
        if self._use_environment_interface:
            return None
        else:
            raise Exception('Not using environment interface')

    def create_environmment_data(self, data):
        if self._use_environment_interface:
            return None
        else:
            raise Exception('Not using environment interface')

    def delete_environment_data(self, uuids):
        if self._use_environment_interface:
            return None
        else:
            raise Exception('Not using environment interface')

    def push_environment_manifest(self):
        if self._use_environment_interface:
            return None
        else:
            raise Exception('Not using environment interface')

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

    def get_available_applications(self):
        if self._use_application_interface:
            response = self._get_app_options_srv()
            return response.options
        else:
            raise Exception('Not using application interface')

    def __program_changed_cb(self, trace):
        #TODO need to keep a manifest of all changed nodes
        # self._program_changes_manifest
        pass
