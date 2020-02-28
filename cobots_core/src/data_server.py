#!/usr/bin/env python

import os
import json
import rospy

from std_msgs.msg import String
from cobots_core.msg import UpdateData, Version

from cobots_core.srv import GetData, GetDataRequest, GetDataResponse
from cobots_core.srv import SetData, SetDataRequest, SetDataResponse
from cobots_core.srv import LoadData, LoadDataRequest, LoadDataResponse
from cobots_core.srv import SaveData, SaveDataRequest, SaveDataResponse
from cobots_core.srv import CreateData, CreateDataRequest, CreateDataResponse
from cobots_core.srv import DeleteData, DeleteDataRequest, DeleteDataResponse
from cobots_core.srv import GetOptions, GetOptionsRequest, GetOptionsResponse

from version_tracking.history import *
from cobots_model.program.program import *
import test_programs.debug_prog as debug_prog


class DataServer:

    def __init__(self, app_filepath, default_program='debug_prog'):
        self._application_filepath = app_filepath
        self._application_filename = None

        if default_program == 'debug_prog':
            self._program = debug_prog.generate()
        else:
            self._program = Program()
        self._program.server_cb = self.__program_updated_cb

        self._program_history = History()
        self._program_history.append(HistoryEntry(
            action='initial',
            change_dct=self._program.to_dct(),
            snapshot_dct=Program().to_dct(),
            source='data-server'
        ))

        self._update_program_pub = rospy.Publisher('application/update_program',UpdateData, queue_size=10)
        self._update_env_pub = rospy.Publisher('application/update_environment',UpdateData, queue_size=10)

        self._load_data_srv = rospy.Service('data_server/load_application_data',LoadData,self._load_application_data_cb)
        self._save_data_srv = rospy.Service('data_server/save_application_data',SaveData,self._save_application_data_cb)
        self._get_app_options_srv = rospy.Service('data_server/get_application_options',GetOptions,self._get_app_options_cb)

        self._get_program_data_srv = rospy.Service('data_server/get_program_data',GetData,self._get_program_data_cb)
        self._set_program_data_srv = rospy.Service('data_server/set_program_data',SetData,self._set_program_data_cb)
        self._create_program_data_srv = rospy.Service('data_server/create_program_data',CreateData,self._create_program_data_cb)
        self._delete_program_data_srv = rospy.Service('data_server/delete_program_data',DeleteData,self._delete_program_data_cb)

        self._get_env_data_srv = rospy.Service('data_server/get_environment_data',GetData,self._get_env_data_cb)
        self._set_env_data_srv = rospy.Service('data_server/set_environment_data',SetData,self._set_env_data_cb)
        self._create_env_data_srv = rospy.Service('data_server/create_environmment_data',CreateData,self._create_env_data_cb)
        self._delete_env_data_srv = rospy.Service('data_server/delete_environment_data',DeleteData,self._delete_env_data_cb)

    def _load_application_data_cb(self, request):
        response = LoadDataResponse()
        data_snapshot = self._program.to_dct()

        # Try opening file
        try:
            fin = open(os.path.join(self._application_filepath,request.filename),'r')
            inStr = fin.read()
            fin.close()
        except:
            response.status = False
            response.message = 'Error reading file'
            return response

        # Try parsing file
        try:
            rawData = json.loads(inStr)
            self._program = Program.from_dct(rawData['program'])
            self._program.server_cb = self.__program_updated_cb
            self._environment = rawData['environment'] #TODO fix later
        except:
            response.status = False
            response.message = 'Error json load'
            return response

        # On successful load
        self._application_filename = request.filename
        self._program_history.append(HistoryEntry(
            action='load',
            change_dct=rawData,
            snapshot_dct = data_snapshot,
            source='data-server'
        ))

        self.__push_program_update()

        response.status = True
        return response

    def _save_application_data_cb(self, request):
        response = SaveDataResponse()
        response.status = True

        # make sure a valid file in selected
        if request.use_current_name:
            if self._application_filename == None:
                response.status = False
                response.message = 'Error cannot save to unspecified file'
                return response
            else:
                used_filepath = self._application_filename
        else:
            used_filepath = request.filename

        # Generate packaged representation of data
        outData = {
            'program': self._program.to_dct(),
            'environment': None #TODO fix later
        }

        # Try writing to file
        try:
            fout = open(os.path.join(self._task_filepath,used_filepath),'w')
            fout.write(json.dumps(outData))
            fout.close()
        except:
            response.status = False
            response.message = 'Error writing to file'
            return response

        # On success
        self._application_filename = used_filepath
        return response

    def _get_app_options_cb(self, request):
        response = GetOptionsResponse()
        response.options = [] #TODO read options as files from app directory

    def _get_program_data_cb(self, request):
        errors = []
        response = GetDataResponse()

        if request.all:
            outData = self._program.to_dct()
        else:
            outData = {}
            inData = json.loads(request.data.data)
            for uuid in inData:
                try:
                    outData[uuid] = self._program.cache.get(uuid).to_dct()
                except:
                    errors.append(uuid)

        response.data = json.dumps(outData)
        response.tag = self._program_history.get_current_version().to_ros()
        response.status = len(errors) == 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error getting data'
        return response

    def _set_program_data_cb(self, request):
        response = SetDataResponse()

        if request.tag == self._history.get_current_version:
            inData = json.loads(request.data.data)
            data_snapshot = self._program.to_dct()

            errors = []
            for uuid in inData.keys():
                try:
                    self._program.cache.set(uuid,inData[uuid])
                except:
                    errors.append(uuid)

            if len(errors) > 0:
                pass #TODO need to revert program from history
            else:
                self._program_history.append(HistoryEntry(
                    action='set',
                    change_dct=inData,
                    snapshot_dct = data_snapshot,
                    source='data-server'
                ))

                self.__push_program_update()

            response.status = len(errors) == 0
            response.errors = json.dumps(errors)
            response.message = '' if response.status else 'error setting data'

        else:
            response.status = False
            response.errors = ''
            response.message = 'mismatched version tags'

        return response

    def _create_program_data_cb(self, request):
        response = CreateDataResponse()

        if request.tag == self._history.get_current_version:
            inData = json.loads(request.data.data)
            data_snapshot = self._program.to_dct()

            errors = []
            for field in inData.keys():
                for dct in inData[field]:
                    try:
                        self._program.create(field,dct)
                    except:
                        errors.append(dct['uuid'])

            if len(errors) > 0:
                pass #TODO need to revert program from history
            else:
                self._program_history.append(HistoryEntry(
                    action='create',
                    change_dct=inData,
                    snapshot_dct = data_snapshot,
                    source='data-server'
                ))

                self.__push_program_update()

            response.status = len(errors) == 0
            response.errors = json.dumps(errors)
            response.message = '' if response.status else 'error creating data'

        else:
            response.status = False
            response.errors = ''
            response.message = 'mismatched version tags'

        return response

    def _delete_program_data_cb(self, request):
        response = DeleteDataResponse()

        if request.tag == self._history.get_current_version:
            inData = json.loads(request.data.data)
            data_snapshot = self._program.to_dct()

            errors = []
            for uuid in inData:
                try:
                    if uuid == self._program.uuid:
                        self._program = self.__new_program()
                        break
                    self._program.delete(uuid)
                except:
                    errors.append(uuid)

            if len(errors) > 0:
                pass #TODO need to revert program from history
            else:
                self._program_history.append(HistoryEntry(
                    action='delete',
                    change_dct=inData,
                    snapshot_dct = data_snapshot,
                    source='data-server'
                ))

                self.__push_program_update()

            response.status = len(errors) > 0
            response.errors = json.dumps(errors)
            response.message = '' if response.status else 'error deleting data'

        else:
            response.status = False
            response.errors = ''
            response.message = 'mismatched version tags'

        return response

    def _get_env_data_cb(self, request):
        pass #TODO write env model

    def _set_env_data_cb(self, request):
        pass #TODO write env model

    def _create_env_data_cb(self, request):
        pass #TODO write env model

    def _delete_env_data_cb(self, request):
        pass #TODO write env model

    def __program_updated_cb(self, attribute_trace):
        pass #TODO all program changes pass through here - structure for update environment?

    def __new_program(self):
        self._program = Program()
        self._program.changes_cb = self.__program_updated_cb

    def __push_program_update(self):
        entry = self._history.get_current_entry()

        msg = UpdateData()
        msg.data = entry.snapshot
        msg.action = entry.action
        msg.changes = entry.changes
        msg.currentTag = entry.version_tag.to_ros()
        msg.previousTag = self._history.get_previous_version().to_ros()

        self._update_program_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('data_server')

    app_filepath = rospy.get_param('~app_filepath')

    node = DataServer(app_filepath)
    rospy.spin()
