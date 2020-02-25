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


from cobots_datatypes.program import *
import test_programs.debug_prog as debug_prog




class DataServer:

    def __init__(self, task_filepath, default_program='debug_prog'):
        if default_program == 'debug_prog':
            self._program = debug_prog.generate()
        else:
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

        self._get_data_srv = rospy.Service('data_server/get_program_data',GetData,self._get_data_cb)
        self._set_data_srv = rospy.Service('data_server/set_program_data',SetData,self._set_data_cb)
        self._load_data_srv = rospy.Service('data_server/load_application_data',LoadData,self._load_data_cb)
        self._save_data_srv = rospy.Service('data_server/save_application_data',SaveData,self._save_data_cb)
        self._create_data_srv = rospy.Service('data_server/create_program_data',CreateData,self._create_data_cb)
        self._delete_data_srv = rospy.Service('data_server/delete_program_data',DeleteData,self._delete_data_cb)
