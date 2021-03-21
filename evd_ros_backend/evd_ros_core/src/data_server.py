#!/usr/bin/env python

'''
Data server is the source of program "truth" in EvD.
'''

import os
import json
import rospy
import traceback

from std_msgs.msg import String
from evd_ros_core.msg import UpdateData, Version, ApplicationOption

from evd_ros_core.srv import GetData, GetDataRequest, GetDataResponse
from evd_ros_core.srv import SetData, SetDataRequest, SetDataResponse

from evd_ros_core.srv import LoadData, LoadDataRequest, LoadDataResponse
from evd_ros_core.srv import SaveData, SaveDataRequest, SaveDataResponse
from evd_ros_core.srv import GetOptions, GetOptionsRequest, GetOptionsResponse

from evd_script.program.program import Program
from evd_version_tracking import *

import test_applications.debug_app as debug_app


DEFAULT_APP = 'debug_app'


class DataServer:

    def __init__(self, app_filepath, default_app):
        self._application_filepath = app_filepath
        self._application_meta = None

        if default_app == 'debug_app':
            dct = debug_app.generate()
            self._program = dct["program"]
            self._application_meta = {
                'filename': 'debug_app.json',
                'name': 'Debug Application',
                'description': 'Application developed for testing core package',
                'level': 0,
                'custom': False
            }
            self._default_objs = dct["default_objs"]
        else:
            self._program = Program()
            self._application_meta ={
                'filename': 'untitled.json',
                'name': 'Untitled Program',
                'description': '',
                'level': 0,
                'custom': False
            }
            self._default_objs = []

        self._program.changes_cb = self.__program_updated_cb

        self._program_history = History()
        self._program_history.append(HistoryEntry(
            action='initial',
            change_dct=self._program.to_dct(),
            snapshot_dct=Program().to_dct(),
            source='data-server'
        ))

        self._update_prog_pub = rospy.Publisher('data_server/update',UpdateData, queue_size=10, latch=True)

        self._load_app_srv = rospy.Service('data_server/load_application_data',LoadData,self._load_app_cb)
        self._save_app_srv = rospy.Service('data_server/save_application_data',SaveData,self._save_app_cb)
        self._get_app_opts_srv = rospy.Service('data_server/get_application_options',GetOptions,self._get_app_opts_cb)

        self._get_prog_srv = rospy.Service('data_server/get_data',GetData,self._get_prog_cb)
        self._set_prog_srv = rospy.Service('data_server/set_data',SetData,self._set_prog_cb)

        self._get_default_objs_srv = rospy.Service('data_server/get_default_objects',GetData,self._get_default_objs_cb)

    def _load_app_cb(self, request):
        response = LoadDataResponse()
        program_snapshot = self._program.to_dct()

        # Try opening file
        try:
            fin = open(os.path.join(self._application_filepath, request.filename),'r')
            inStr = fin.read()
            fin.close()
        except:
            traceback.print_exc()
            response.status = False
            response.message = 'Error reading in program file'
            return response

        # Try parsing file
        try:
            rawData = json.loads(inStr)
            self._program.remove_from_cache()
            self._program = Program.from_dct(rawData["program"])
            self._program.changes_cb = self.__program_updated_cb
            self._default_objs = NodeParser(rawData["default_objs"])
        except:
            traceback.print_exc()
            response.status = False
            response.message = 'Error on program json load'
            return response

        # On successful load
        self._program_history.append(HistoryEntry(
            action='load',
            change_dct=rawData,
            snapshot_dct = program_snapshot,
            source='data-server'
        ))

        self._application_meta ={
            'filename': request.filename,
            'name': request.name,
            'description': request.description,
            'level': request.level,
            'custom': request.custom
        }

        self._push_program_update()

        response.status = True
        return response

    def _save_app_cb(self, request):
        response = SaveDataResponse()
        response.status = True

        # make sure a valid file in selected
        if request.use_current_info:
            if self._application_meta['filename'] == None:
                response.status = False
                response.message = 'Error cannot save to unspecified file'
                return response
            else:
                used_filepath = self._application_meta['filename']
        else:
            used_filepath = request.filename

        # Generate packaged representation of data
        outData = {"program": self._program.to_dct(), "default_objs": self._default_objs}

        # Try writing to file
        try:
            fout = open(os.path.join(self._application_filepath, used_filepath),'w')
            fout.write(json.dumps(outData, indent=4, sort_keys=True))
            fout.close()
        except:
            traceback.print_exc()
            response.status = False
            response.message = 'Error writing to progam file'
            return response

        # On success
        if not request.use_current_info:
            self._application_meta = {
                'filename': request.filename,
                'name': request.name,
                'description': request.description,
                'level': request.level,
                'custom': True
            }

            # Update meta file listing
            # Try opening meta file
            try:
                fin = open(os.path.join(self._application_filepath,'meta.json'),'r')
                inStr = fin.read()
                fin.close()
            except:
                traceback.print_exc()
                response.status = False
                response.message = 'Error reading in meta file'
                return response

            # Try parsing file
            try:
                metaData = json.loads(inStr)
            except:
                traceback.print_exc()
                response.status = False
                response.message = 'Error json load meta file'
                return response

            idx = None
            for i in range(0,len(metaData['options'])):
                if request.filename == metaData['options'][i]['filename']:
                    idx = i
                    break

            if idx != None:
                metaData['options'][i] = self._application_meta
            else:
                metaData['options'].append(self._application_meta)

            try:
                fout = open(os.path.join(self._application_filepath,'meta.json'),'w')
                fout.write(json.dumps(metaData, indent=4, sort_keys=True))
                fout.close()
            except:
                traceback.print_exc()
                response.status = False
                response.message = 'Error writing to meta file'
                return response

        return response

    def _get_app_opts_cb(self, request):
        response = GetOptionsResponse()

        # Try opening meta file
        try:
            fin = open(os.path.join(self._application_filepath,'meta.json'),'r')
            inStr = fin.read()
            fin.close()
        except:
            traceback.print_exc()
            response.status = False
            response.message = 'Error reading file'
            return response

        # Try parsing file
        try:
            metaData = json.loads(inStr)
        except:
            traceback.print_exc()
            response.status = False
            response.message = 'Error json load'
            return response

        # Process meta data
        response.title = metaData['name']
        response.description = metaData['description']
        response.currently_loaded_filename = self._application_meta['filename']

        for option in metaData['options']:
            appOpt = ApplicationOption()
            appOpt.filename = option['filename']
            appOpt.name = option['name']
            appOpt.description = option['description']
            appOpt.level = option['level']
            appOpt.custom = option['custom']

            response.options.append(appOpt)

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

        response.data = json.dumps(outData, indent=4, sort_keys=True)
        response.tag = self._program_history.get_current_version().to_ros()
        response.status = len(errors) == 0
        response.errors = json.dumps(errors, indent=4, sort_keys=True)
        response.message = '' if response.status else 'error getting data'
        return response

    def _set_prog_cb(self, request):
        response = SetDataResponse()

        if request.tag == self._program_history.get_current_version:
            inData = json.loads(request.data.data)
            data_snapshot = self._program.to_dct()

            errors = []
            for uuid in inData.keys():
                try:
                    #TODO fix this because this is not going to work
                    self._program.cache.set(uuid,inData[uuid])
                except:
                    traceback.print_exc()
                    errors.append(uuid)

            if len(errors) > 0:
                self._program.remove_from_cache()
                self._program = Program.from_dct(data_snapshot)
                self._program.changes_cb = self.__program_updated_cb
            else:
                self._program_history.append(HistoryEntry(
                    action='set',
                    change_dct=inData,
                    snapshot_dct = data_snapshot,
                    source='data-server'
                ))

                self._push_program_update()

            response.status = len(errors) == 0
            response.errors = json.dumps(errors, indent=4, sort_keys=True)
            response.message = '' if response.status else 'error setting data, reverted to previous program state'

        else:
            response.status = False
            response.errors = ''
            response.message = 'mismatched version tags'

        return response

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

    def __program_updated_cb(self, attribute_trace):
        pass # maybe implement a repair routine here?

    def _get_default_objs_cb(self, request):
        errors = []
        response = GetDataResponse()

        outData = {}
        if request.all:
            for key in self._default_objs.keys():
                outData[key] = [e.to_dct() for e in self._default_objs[key]]
        else:
            inData = json.loads(request.data.data)
            for group in inData:
                selected = self._default_objs[group] if group in self._default_objs.keys() else []
                outData[group] = [e.to_dct() for e in selected]

        response.data = json.dumps(outData, indent=4, sort_keys=True)
        response.tag = VersionTag('data_server').to_ros()
        response.status = len(errors) == 0
        response.errors = json.dumps(errors, indent=4, sort_keys=True)
        response.message = '' if response.status else 'error getting data'
        return response

    def spin(self):
        # give nodes time before publishing initial state
        # the topic latches message but as a best practice, it shouldn't rush
        # all other nodes with the program data until general setup is complete

        rospy.sleep(10)
        self._push_program_update()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('data_server')

    app_filepath = rospy.get_param('~app_filepath')
    default_app = rospy.get_param('~default_app',DEFAULT_APP)

    node = DataServer(app_filepath,default_app)
    node.spin()
