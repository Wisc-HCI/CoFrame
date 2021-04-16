'''
Data client interface wraps the interaction with the data server for any other node.

This is needed as EvD has a "heavy" model on each client (a full copy of the program tree).
Doing it this way makes it nice to edit locally before commiting changes but will make the
actual change manifest much harder to enforce manually. Hence, wrapping that behavior
for the user in this node.

We also support application level interface for loading / saving whole programs if
needed by the node.

Note: Currently the setting changes back to data server is broken.
'''


import json
import rospy
import traceback

from std_msgs.msg import String
from evd_ros_core.msg import UpdateData, Version

from evd_ros_core.srv import GetData, GetDataRequest, GetDataResponse
from evd_ros_core.srv import SetData, SetDataRequest, SetDataResponse
from evd_ros_core.srv import LoadData, LoadDataRequest, LoadDataResponse
from evd_ros_core.srv import SaveData, SaveDataRequest, SaveDataResponse
from evd_ros_core.srv import GetOptions, GetOptionsRequest, GetOptionsResponse

from evd_script import *
from evd_version_tracking import *


class DataClientInterface(object):

    def __init__(self, use_application_interface=False, on_program_update_cb=None):
        self._server_has_updated = False

        self._use_application_interface = use_application_interface
        self._cache = get_evd_cache_obj()

        if self._use_application_interface:
            self._load_app_srv = rospy.ServiceProxy('data_server/load_application_data',LoadData)
            self._save_app_srv = rospy.ServiceProxy('data_server/save_application_data',SaveData)
            self._get_app_options_srv = rospy.ServiceProxy('data_server/get_application_options',GetOptions)

        self._program = None
        self._default_objs= []
        self._program_verison = None
        self._program_changes_manifest = []
        self._on_program_update_cb = on_program_update_cb

        self._update_sub = rospy.Subscriber('data_server/update',UpdateData, self._update_cb)

        self._get_srv = rospy.ServiceProxy('data_server/get_data',GetData)
        self._set_srv = rospy.ServiceProxy('data_server/set_data',SetData)
        self._get_default_objs_srv = rospy.ServiceProxy('data_server/get_default_objects',GetData)

    '''
    Application Interface
    '''

    def load_application(self, filename, name, description, level, custom=True):
        if self._use_application_interface:
            response = self._load_app_srv(filename, name, description, level, custom)
            return response.status, response.message
        else:
            raise Exception('Not using application interface')

    def save_application(self, use_current_info=True, filename='untitled.json', name='', description='', level=0):
        if self._use_application_interface:
            response = self._save_app_srv(use_current_info, filename, name, description, level)
            return response.status, response.message
        else:
            raise Exception('Not using application interface')

    def get_application_options(self):
        if self._use_application_interface:
            response = self._get_app_options_srv()
            return response
        else:
            raise Exception('Not using application interface')

    '''
    Program Interface
    '''

    @property
    def program(self):
        return self._program

    @property
    def has_local_changes(self):
        return len(self._program_changes_manifest) > 0

    def get_data(self, fetch=False):
        if fetch:
            response = self._get_program_srv(True,'')
            if response.status:
                self._program = Program.from_dct(json.loads(response.data))
                self._program.late_construct_update()
                self._program.changes_cb = self.__program_changed_cb
                self._program_verison = VersionTag.from_ros(response.tag)
                return self._program
            else:
                raise Exception(response.message)
        else:
            return self._program

    def push_changes(self):
        # set service with manifest
        # TODO  determine manifest structure
        return None

    def get_default_objects(self, fetch=False):
        if fetch:
            response = self._get_default_objs_srv(True,'')
            if response.status:
                self._default_objs = json.loads(response.data)
                return self._default_objs
            else:
                raise Exception(response.message)
        else:
            return self._default_objs

    '''
    Private - Utility
    '''

    def _update_cb(self, msg):
        #TODO need to do a smarter version of this where if our tag matches previous tag then only perform the update to current tag
        #TODO need to push these changes through the program update callback (just the changes!)
        try:
            self._program = Program.from_dct(json.loads(msg.data))
            self._program.late_construct_update()
            self._program.changes_cb = self.__program_changed_cb
            self._program_verison = VersionTag.from_ros(msg.currentTag)
        except:
            traceback.print_exc()
            return #Error parsing, ignore for now

        self._server_has_updated = True

        if self._on_program_update_cb != None:
            self._on_program_update_cb()

    def __program_changed_cb(self, trace):
        #TODO need to keep a manifest of all changed nodes
        # self._program_changes_manifest
        pass

    @property
    def server_has_updated(self):
        val = self._server_has_updated
        self._server_has_updated = False
        return val

    '''
    Public Utilities
    '''

    @property
    def cache(self):
        return self._cache
