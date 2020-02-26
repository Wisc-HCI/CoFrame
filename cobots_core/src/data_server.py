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

from cobots_model.program.program import *
import test_programs.debug_prog as debug_prog


class DataServer:

    def __init__(self, task_filepath, default_program='debug_prog'):
        if default_program == 'debug_prog':
            self._program = debug_prog.generate()
        else:
            self._program = Program()
        self._program.server_cb = self.__program_updated_cb

        self._version_tag = self.__create_version_tag()
        self._history = []
        self.__append_history({
            'action': 'initial',
            'change': self._program.to_dct(),
            'version_tag': self._version_tag,
            'snapshot': None
        })
        self._task_filepath = task_filepath

        self._update_program_pub = rospy.Publisher('application/update_program',UpdateData, queue_size=10)
        self._update_env_pub = rospy.Publisher('application/update_environment',UpdateData, queue_size=10)

        self._load_data_srv = rospy.Service('data_server/load_application_data',LoadData,self._load_application_data_cb)
        self._save_data_srv = rospy.Service('data_server/save_application_data',SaveData,self._save_application_data_cb)

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

        try:
            fin = open(os.path.join(self._task_filepath,request.filename),'r')
            inStr = fin.read()
            fin.close()
        except:
            response.status = False
            response.message = 'Error reading file'
            return response

        try:
            rawData = json.loads(inStr)
            self._program = Program.from_dct(rawData['program'])
            self._program.server_cb = self.__program_updated_cb
            self._environment = rawData['environment'] #TODO fix later
            self._version_tag = self.__create_version_tag()
        except:
            response.status = False
            response.message = 'Error json load'
            return response
        self.__append_history({
            'action': 'load',
            'change': rawData,
            'version_tag': self._version_tag,
            'snapshot': data_snapshot
        })

        response.status = True
        return response

    def _save_application_data_cb(self, request):
        response = SaveDataResponse()
        response.status = True

        outData = {
            'program': self._program.to_dct(),
            'environment': None #TODO fix later
        }

        try:
            fout = open(os.path.join(self._task_filepath,request.filename),'w')
            fout.write(json.dumps(outData))
            fout.close()
        except:
            response.status = False
            response.message = 'Error writing to file'

        return response

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
        response.tag.source = self._version_tag['source']
        response.tag.timestamp = rospy.Time.from_sec(self._version_tag['timestamp'])
        response.status = len(errors) == 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error getting data'
        return response

    def _set_program_data_cb(self, request):
        response = SetDataResponse()

        inData = json.loads(request.data.data)
        data_snapshot = self._program.to_dct()

        errors = []
        for uuid in inData.keys():
            try:
                self._program.cache.set(uuid,inData[uuid])
            except:
                errors.append(uuid)

        self._version_tag = {
            'timestamp': request.tag.timestamp.to_sec(),
            'source': request.tag.source
        }
        self.__append_history({
            'action': 'set',
            'change': inData,
            'version_tag': self._version_tag,
            'snapshot': data_snapshot
        })

        response.status = len(errors) == 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error setting data'
        return response

    def _create_program_data_cb(self, request):
        response = CreateDataResponse()

        inData = json.loads(request.data.data)
        data_snapshot = self._program.to_dct()

        errors = []
        for field in inData.keys():
            for dct in inData[field]:
                try:
                    self._program.create(field,dct)
                except:
                    errors.append(dct['uuid'])

        self._version_tag = {
            'timestamp': request.tag.timestamp.to_sec(),
            'source': request.tag.source
        }
        self.__append_history({
            'action': 'create',
            'change': inData,
            'version_tag': self._version_tag,
            'snapshot': data_snapshot
        })

        response.status = len(errors) == 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error creating data'
        return response

    def _delete_program_data_cb(self, request):
        response = DeleteDataResponse()

        inData = json.loads(request.data.data)
        data_snapshot = self._program.to_dct()

        errors = []
        for uuid in inData:
            try:
                self._program.delete(uuid)
            except:
                errors.append(uuid)

        self._version_tag = {
            'timestamp': request.tag.timestamp.to_sec(),
            'source': request.tag.source
        }
        self.__append_history({
            'action': 'delete',
            'change': inData,
            'version_tag': self._version_tag,
            'snapshot': data_snapshot
        })

        response.status = len(errors) > 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error deleting data'
        return response

    def _get_env_data_cb(self, request):
        pass #TODO write env model

    def _set_env_data_cb(self, request):
        pass #TODO write env model

    def _create_env_data_cb(self, request):
        pass #TODO write env model

    def _delete_env_data_cb(self, request):
        pass #TODO write env model

    def __append_history(self, item):
        self._history.append(item)
        if len(self._history) > 10:
            self._history.pop(0)

    def __create_version_tag(self):
        return {
            'timestamp': rospy.Time.now().to_sec(),
            'source': 'data_server'
        }

    def __program_updated_cb(self, attribute_trace):
        pass #TODO all program changes pass through here - structure for update


if __name__ == '__main__':
    rospy.init_node('data_server')

    task_filepath = rospy.get_param('~task_filepath')

    node = DataServer(task_filepath)
    rospy.spin()
