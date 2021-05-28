'''
Data client interface wraps the interaction with the data server for any other node.

This is needed as EvD has a "heavy" model on each client (a full copy of the program tree).
Doing it this way makes it nice to edit locally before commiting changes but will make the
actual change manifest much harder to enforce manually. Hence, wrapping that behavior
for the user in this node.

We also support application level interface for loading / saving whole programs if
needed by the node. This is an optional feature that can be enabled.

TODO Currently the setting changes back to data server is broken / needs implementation.
TODO Update history get partials to use new message format
TODO Make use of additional attributes in UpdateData on has_changes sub
'''


import json
import rospy
import traceback

from evd_ros_core.msg import UpdateData

from evd_ros_core.srv import GetHistory
from evd_ros_core.srv import GetData, SetData
from evd_ros_core.srv import LoadData, SaveData,GetOptions

from evd_script import Program, NodeParser
from evd_script.cache import get_evd_cache_obj
from evd_version_tracking import History, HistoryEntry, VersionTag


class DataClientInterface(object):

    def __init__(self, use_application_interface=False, sub_to_update=True, on_program_update_cb=None, store_program=True, track_local_changes=True):
        self._server_has_updated = False
        self._store_program = store_program
        self.sub_to_update = sub_to_update
        self.track_local_changes = track_local_changes

        self._use_application_interface = use_application_interface
        self._cache = get_evd_cache_obj()

        if self._use_application_interface:
            self._load_app_srv = rospy.ServiceProxy('data_server/load_application_data',LoadData)
            self._save_app_srv = rospy.ServiceProxy('data_server/save_application_data',SaveData)
            self._get_app_options_srv = rospy.ServiceProxy('data_server/get_application_options',GetOptions)

        self._program = None
        self._history = None
        self._program_verison = None
        self._program_changes_manifest = []
        self._on_program_update_cb = on_program_update_cb

        if self.sub_to_update:
            self._update_sub = rospy.Subscriber('data_server/update',UpdateData, self._update_cb)
        self._has_changes_pub = rospy.Subscriber('data_server/has_changes',UpdateData, self._has_changes_cb)
        
        self._get_program_srv = rospy.ServiceProxy('data_server/get_program',GetData)
        self._set_program_srv = rospy.ServiceProxy('data_server/set_program',SetData)
        self._get_history_srv = rospy.ServiceProxy('data_server/get_history',GetHistory)
        self._get_uuids_srv = rospy.ServiceProxy('data_server/get_uuids',GetData)

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
    def server_has_updated(self):
        val = self._server_has_updated
        self._server_has_updated = False
        return val

    @property
    def cache(self):
        if self._store_program:
            return self._cache
        else:
            raise Exception('Not storing program')

    @property
    def program(self):
        if self._store_program:
            return self._program
        else:
            raise Exception('Not storing program')

    @property
    def has_local_changes(self):
        if self._store_program:
            return len(self._program_changes_manifest) > 0
        else:
            raise Exception('Not storing program')

    def get_program(self, fetch=False):
        if fetch:
            response = self._get_program_srv(True,'')
            if response.status:
                program = Program.from_dct(json.loads(response.data))
                program.late_construct_update()

                if self._store_program:
                    self._program = program
                    self._program.changes_cb = self.__program_changed_cb
                    self._program_verison = VersionTag.from_ros(response.tag)

                return program
            else:
                raise Exception(response.message)
        elif self._store_program:
            return self._program
        else:
            raise Exception('Not storing program')

    def get_program_partials(self, uuids=[]):
        response = self._get_program_srv(False,json.dumps(uuids))

        if not response.status:
            return False, json.loads(response.errors)

        raw = json.loads(response.data)
        partials = { key: NodeParser(raw[key], no_cache=True) for key in raw.keys()}

        return True, partials

    def push_changes(self):
        if self._store_program:
            # set service with manifest
            # TODO  determine manifest structure
            return None
        else:
            raise Exception('Not storing program')

    def _update_cb(self, msg):
        if self._store_program:
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

    def __program_changed_cb(self, trace):
        # This only runs if program saved
        '''
        trace = [{
            type: node.type
            uuid: node.uuid
            attribute: 'property' or none
            verb: [set, add, delete, callback]
            child_uuid: none (if set, callback) or child.uuid (if add, delete)
        },...]
        '''

        #TODO need to keep a manifest of all changed nodes
        # self._program_changes_manifest
        pass

    def _has_changes_cb(self, msg):
        #TODO do things with action, changes, and tags
        #NOTE data is purposefully empty here
        self._server_has_updated = True
        if self._on_program_update_cb != None:
            self._on_program_update_cb()

    def get_history(self, fetch=False):
        if fetch:
            response = self._get_history_srv(True,'')
            if response.status:
                history = History.from_dct(json.loads(response.data))

                if self._store_program:
                    self._history = history

                return history
            else:
                raise Exception(response.message)
        elif self._store_program:
            return self._history
        else:
            raise Exception('Not storing program')

    def get_history_partials(self, tags=[]):
        #TODO this needs to be updated
        response = self._get_history_srv(False,json.dumps(tags))

        if not response.status:
            return False, json.loads(response.errors)

        raw = json.loads(response.data)
        partials = { key: HistoryEntry.from_dct(raw[key]) for key in raw.keys() }

        return True, partials

    def _create_manifest_entry(self):
        return {
            
        }
