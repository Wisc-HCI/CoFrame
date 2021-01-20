#!/usr/bin/env python

import os
import json
import rospy

from std_msgs.msg import String
from evd_ros_core.msg import UpdateData, Version

from evd_ros_core.srv import GetData, GetDataRequest, GetDataResponse
from evd_ros_core.srv import SetData, SetDataRequest, SetDataResponse

from evd_ros_core.srv import LoadData, LoadDataRequest, LoadDataResponse
from evd_ros_core.srv import SaveData, SaveDataRequest, SaveDataResponse
from evd_ros_core.srv import GetOptions, GetOptionsRequest, GetOptionsResponse

from evd_script.program.program import *
from evd_script.environment.environment import *
from evd_script.version_tracking.history import *

import test_applications.debug_app as debug_app


DEFAULT_APP = 'debug_app'


#TODO environment handling needs to be done

class DataServer:

    def __init__(self, app_filepath, default_app):
        self._application_filepath = app_filepath
        self._application_filename = None

        if default_app == 'debug_app':
            app = debug_app.generate()
            self._program = app['program']
            self._environment = app['environment']
        else:
            self._program = Program()
            self._environment = Environment()

        self._program_history = History()
        self._program_history.append(HistoryEntry(
            action='initial',
            change_dct=self._program.to_dct(),
            snapshot_dct=Program().to_dct(),
            source='data-server'
        ))

        self._environment_history = History()
        self._environment_history.append(HistoryEntry(
            action='initial',
            change_dct=self._environment.to_dct(),
            snapshot_dct=Environment().to_dct(),
            source='data-server'
        ))

        self._update_prog_pub = rospy.Publisher('data_server/update_program',UpdateData, queue_size=10, latch=True)
        self._update_env_pub = rospy.Publisher('data_server/update_environment',UpdateData, queue_size=10, latch=True)

        self._load_app_srv = rospy.Service('data_server/load_application_data',LoadData,self._load_app_cb)
        self._save_app_srv = rospy.Service('data_server/save_application_data',SaveData,self._save_app_cb)
        self._get_app_opts_srv = rospy.Service('data_server/get_application_options',GetOptions,self._get_app_opts_cb)

        self._get_prog_srv = rospy.Service('data_server/get_program_data',GetData,self._get_prog_cb)
        self._set_prog_srv = rospy.Service('data_server/set_program_data',SetData,self._set_prog_cb)

        self._get_env_srv = rospy.Service('data_server/get_environment_data',GetData,self._get_env_cb)
        self._set_env_srv = rospy.Service('data_server/set_environment_data',SetData,self._set_env_cb)

    def _load_app_cb(self, request):
        response = LoadDataResponse()
        program_snapshot = self._program.to_dct()
        environment_snapshot = self._environment.to_dct()

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
            self._environment = Environment.from_dct(rawData['environment'])
            self._environment.server_cb = self._environent_updated_cb
        except:
            response.status = False
            response.message = 'Error json load'
            return response

        # On successful load
        self._application_filename = request.filename

        self._program_history.append(HistoryEntry(
            action='load',
            change_dct=rawData['program'],
            snapshot_dct = program_snapshot,
            source='data-server'
        ))

        self._environment_history.append(HistoryEntry(
            action='load',
            change_dct=rawData['environment'],
            snapshot_dct = environment_snapshot,
            source='data-server'
        ))

        self._push_program_update()
        self._push_environment_update()

        response.status = True
        return response

    def _save_app_cb(self, request):
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
            'environment': self._environment.to_dct()
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

    def _get_app_opts_cb(self, request):
        response = GetOptionsResponse()

        response.options = [f for f in os.listdir(self._application_filepath) if os.isfile(os.join(self._application_filepath,f))]

        return response

    def _get_prog_cb(self, request):
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

    def _set_prog_cb(self, request):
        response = SetDataResponse()

        #TODO figure out the best way to update both the program and environment
        '''
        if request.tag == self._program_history.get_current_version:
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

                self._push_program_update()

            response.status = len(errors) == 0
            response.errors = json.dumps(errors)
            response.message = '' if response.status else 'error setting data'

        else:
            response.status = False
            response.errors = ''
            response.message = 'mismatched version tags'
        '''

        return response

    def _get_env_cb(self, request):
        errors = []
        response = GetDataResponse()

        if request.all:
            outData = self._environment.to_dct()
        else:
            outData = {}
            inData = json.loads(request.data.data)
            for uuid in inData:
                try:
                    outData[uuid] = self._environment.cache.get(uuid).to_dct()
                except:
                    errors.append(uuid)

        response.data = json.dumps(outData)
        response.tag = self._environment_history.get_current_version().to_ros()
        response.status = len(errors) == 0
        response.errors = json.dumps(errors)
        response.message = '' if response.status else 'error getting data'
        return response

    def _set_env_cb(self, request):
        pass #TODO write env model

    def _push_program_update(self):
        entry = self._program_history.get_current_entry()

        msg = UpdateData()
        msg.data = json.dumps(self._program.to_dct())
        msg.action = entry.action
        msg.changes = json.dumps(entry.changes)
        msg.currentTag = entry.version.to_ros()

        prevTag = self._program_history.get_previous_version()
        msg.previousTag = prevTag.to_ros() if prevTag != None else Version(rospy.Duration(0),'','data-server')

        self._update_prog_pub.publish(msg)

    def _push_environment_update(self):
        entry = self._environment_history.get_current_entry()

        msg = UpdateData()
        msg.data = json.dumps(self._environment.to_dct())
        msg.action = entry.action
        msg.changes = json.dumps(entry.changes)
        msg.currentTag = entry.version.to_ros()

        prevTag = self._environment_history.get_previous_version()
        msg.previousTag = prevTag.to_ros() if prevTag != None else Version(rospy.Duration(0),'','data-server')

        self._update_env_pub.publish(msg)

    def spin(self):
        # give nodes time before publishing initial state
        # the topic latches message but as a best practice, it shouldn't rush
        # all other nodes with the program data until general setup is his
        # complete.
        rospy.sleep(30)
        self._push_program_update()
        self._push_environment_update()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('data_server')

    app_filepath = rospy.get_param('~app_filepath')
    default_app = rospy.get_param('~default_app',DEFAULT_APP)

    node = DataServer(app_filepath,default_app)
    node.spin()
