#!/usr/bin/env python

'''
The data server creates a link between the application and the data backend of
the ROS interface.
'''

import os
import json
import rospy

from std_msgs.msg import String

from cobots_core.srv import GetData, GetDataRequest, GetDataResponse
from cobots_core.srv import LoadData, LoadDataRequest, LoadDataResponse
from cobots_core.srv import SaveData, SaveDataRequest, SaveDataResponse


class DataServer:

    def __init__(self, task_filepath):
        self._task = {}
        self._task_filepath = task_filepath

        self._update_sub = rospy.Subscriber('application/update',String,self._update_cb)

        self._get_data_srv = rospy.Service('data_server/get_data',GetData,self._get_data_cb)
        self._load_data_srv = rospy.Service('data_server/load_data',LoadData,self._load_data_cb)
        self._save_data_srv = rospy.Service('data_server/save_data',SaveData,self._save_data_cb)

    def _update_cb(self, msg):
        self._task = json.loads(msg.data)

    def _get_data_cb(self, request):
        inData = json.loads(request.data.data)
        outData = {}

        for data_type in inData.keys():
            outData[data_type] = {}

            for data_id in inData[data_type]:
                if data_type in self._task.keys() and data_id in self._task[data_type].keys():
                    outData[data_type][data_id] = self._task[data_type][data_id]
                else:
                    outData[data_type][data_id] = {'error': True}

        response = GetDataResponse()
        response.data = String(json.dumps(outData))
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
            self._task = inData

        response.data = json.dumps()
        response.status = True
        return response

    def _save_data_cb(self, request):
        response = SaveDataResponse()
        response.status = True

        try:
            fout = open(os.path.join(self._task_filepath,request.filename),'w')
            fout.write(json.dumps(self._task))
            fout.close()
        except:
            response.status = False
            response.message = 'Error writing to file'

        return response


if __name__ == '__main__':
    rospy.init_node('data_server')

    task_filepath = rospy.get_param('~task_filepath')

    node = DataServer(task_filepath)
    rospy.spin()
