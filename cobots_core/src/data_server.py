#!/usr/bin/env python

'''
The data server creates a link between the application and the data backend of
the ROS interface.
'''

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

from cobots_datatypes.task import *
from cobots_datatypes.program import *
from cobots_datatypes.abstract import *
from cobots_datatypes.geometry import *
from cobots_datatypes.location import *
from cobots_datatypes.waypoint import *
from cobots_datatypes.primitive import *
from cobots_datatypes.trajectory import *


class DataServer:

    def __init__(self, task_filepath):
        self._program = Program()
        self._version_tag = self._create_version_tag()
        self._history = []
        self._append_history({
            'action': 'initial',
            'change': self._program.to_dct(),
            'version_tag': self._version_tag,
            'snapshot': None
        })
        self._task_filepath = task_filepath

        self._update_pub = rospy.Publisher('application/update',UpdateData, queue_size=10)

        self._get_data_srv = rospy.Service('data_server/get_data',GetData,self._get_data_cb)
        self._set_data_srv = rospy.Service('data_server/set_data',SetData,self._set_data_cb)
        self._load_data_srv = rospy.Service('data_server/load_data',LoadData,self._load_data_cb)
        self._save_data_srv = rospy.Service('data_server/save_data',SaveData,self._save_data_cb)
        self._create_data_srv = rospy.Service('data_server/create_data',CreateData,self._create_data_cb)
        self._delete_data_srv = rospy.Service('data_server/delete_data',DeleteData,self._delete_data_cb)

    def _get_data_cb(self, request):
        errors = {}
        response = GetDataResponse()

        if request.all:
            response.data = json.dumps(self._program.to_dct())
        else:
            outData = {}
            inData = json.loads(request.data.data)
            for field in inData.keys():
                outData[field] = {}
                for did in inData[field]:
                    try:
                        outData[field][did] = self._program.get(field,did).to_dct()
                    except:
                        if not field in errors.keys():
                            errors[field] = []
                        errors[field].append(did)

        response.tag.source = self._version_tag['source']
        response.tag.timestamp = rospy.Time.from_sec(self._version_tag['timestamp'])
        response.status = len(errors.keys()) == 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error getting data'
        return response

    def _load_data_cb(self, request):
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
            self._program = Program.from_dct(rawData)
            self._version_tag = self._create_version_tag()
        except:
            response.status = False
            response.message = 'Error json load'
            return response
        self._append_history({
            'action': 'load',
            'change': rawData,
            'version_tag': self._version_tag,
            'snapshot': data_snapshot
        })

        response.status = True
        return response

    def _save_data_cb(self, request):
        response = SaveDataResponse()
        response.status = True

        try:
            fout = open(os.path.join(self._task_filepath,request.filename),'w')
            fout.write(json.dumps(self._program.to_dct()))
            fout.close()
        except:
            response.status = False
            response.message = 'Error writing to file'

        return response

    def _set_data_cb(self, request):
        response = SetDataResponse()

        inData = json.loads(request.data.data)
        data_snapshot = self._program.to_dct()

        errors = {}
        for field in inData.keys():
            for did in inData[field].keys():
                try:
                    self._program.set(field, did, inData[field][did])
                except:
                    if not field in errors.keys():
                        errors[field] = []
                    errors[field].append(did)


        self._version_tag = {
            'timestamp': request.tag.timestamp.to_sec(),
            'source': request.tag.source
        }
        self._append_history({
            'action': 'set',
            'change': inData,
            'version_tag': self._version_tag,
            'snapshot': data_snapshot
        })

        response.status = len(errors) == 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error setting data'
        return response

    def _create_data_cb(self, request):
        response = CreateDataResponse()

        inData = json.loads(request.data.data)
        data_snapshot = self._program.to_dct()

        errors = {}
        for field in inData.keys():
            for did in inData[field].keys():
                try:
                    self._program.create(field, did, inData[field][did])
                except:
                    if not field in errors.keys():
                        errors[field] = []
                    errors[field].append(did)

        self._version_tag = {
            'timestamp': request.tag.timestamp.to_sec(),
            'source': request.tag.source
        }
        self._append_history({
            'action': 'create',
            'change': inData,
            'version_tag': self._version_tag,
            'snapshot': data_snapshot
        })

        response.status = len(errors) == 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error creating data'
        return response

    def _delete_data_cb(self, request):
        response = DeleteDataResponse()

        inData = json.loads(request.data.data)
        data_snapshot = self._program.to_dct()

        errors = {}
        for field in inData.keys():
            for did in inData[field].keys():
                try:
                    self._program.delete(field, did)
                except:
                    if not field in errors.keys():
                        errors[field] = []
                    errors[field].append(did)

        self._version_tag = {
            'timestamp': request.tag.timestamp.to_sec(),
            'source': request.tag.source
        }
        self._append_history({
            'action': 'delete',
            'change': inData,
            'version_tag': self._version_tag,
            'snapshot': data_snapshot
        })

        response.status = len(errors) > 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error deleting data'
        return response

    def _append_history(self, item):
        self._history.append(item)
        if len(self._history) > 10:
            self._history.pop(0)

    def _create_version_tag(self):
        return {
            'timestamp': rospy.Time.now().to_sec(),
            'source': 'data_server'
        }


if __name__ == '__main__':
    rospy.init_node('data_server')

    task_filepath = rospy.get_param('~task_filepath')

    node = DataServer(task_filepath)
    rospy.spin()
