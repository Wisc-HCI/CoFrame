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
        self._task_filepath = task_filepath

        self._update_pub = rospy.Publisher('application/update',UpdateData, queue_size=10)

        self._get_data_srv = rospy.Service('data_server/get_data',GetData,self._get_data_cb)
        self._set_data_srv = rospy.Service('data_server/set_data',SetData,self._set_data_cb)
        self._load_data_srv = rospy.Service('data_server/load_data',LoadData,self._load_data_cb)
        self._save_data_srv = rospy.Service('data_server/save_data',SaveData,self._save_data_cb)
        self._create_data_srv = rospy.Service('data_server/create_data',CreateData,self._create_data_cb)
        self._delete_data_srv = rospy.Service('data_server/delete_data',DeleteData,self._delete_data_cb)

    def _get_data_cb(self, request):
        inData = json.loads(request.data.data)
        outData = {}

        for data_type in inData.keys():
            outData[data_type] = {}

            for data_id in inData[data_type]:
                if data_type in self._task_raw.keys() and data_id in self._task_raw[data_type].keys():
                    outData[data_type][data_id] = self._task_raw[data_type][data_id]
                else:
                    outData[data_type][data_id] = {'error': True}

        response = GetDataResponse()
        response.data = String(json.dumps(outData))
        response.timestamp = self._timestamp
        return response

    def _load_data_cb(self, request):
        response = LoadDataResponse()

        try:
            fin = open(os.path.join(self._task_filepath,request.filename),'r')
            inStr = fin.read()
            fin.close()
        except:
            response.status = False
            response.message = 'Error reading file'
            return response

        try:
            inData = json.loads(inStr)
        except:
            response.status = False
            response.message = 'Error json load'
            return response

        if request.update_cache:
            self._task_raw = inData
            self._timestamp = rospy.Time.now()

        response.data = json.dumps()
        response.status = True
        return response

    def _save_data_cb(self, request):
        response = SaveDataResponse()
        response.status = True

        try:
            fout = open(os.path.join(self._task_filepath,request.filename),'w')
            fout.write(json.dumps(program_raw))
            fout.close()
        except:
            response.status = False
            response.message = 'Error writing to file'

        return response

    def _set_data_cb(self, request):
        pass

    def _create_data_cb(self, request):
        pass

    def _delete_data_cb(self, request):
        pass


if __name__ == '__main__':
    rospy.init_node('data_server')

    task_filepath = rospy.get_param('~task_filepath')

    node = DataServer(task_filepath)
    rospy.spin()
