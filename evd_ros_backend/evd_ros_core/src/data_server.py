#!/usr/bin/env python3

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
from evd_ros_core.srv import GetHistory, GetHistoryRequest, GetHistoryResponse

from evd_script.program.program import Program
from evd_script.cache import get_evd_cache_obj
from evd_version_tracking import HistoryEntry, History, VersionTag


class DataServer:

    def __init__(self, app_filepath):
        self._application_filepath = app_filepath

        # Setup Program Data
        self._program = Program()
        self._program.changes_cb = self.__program_updated_cb
        self._program_cache = get_evd_cache_obj()

        self._program_history = History()
        self._program_history.append(HistoryEntry(
            action='initial',
            change_dct=self._program.to_dct(),
            snapshot_dct=Program().to_dct(),
            source='data-server'
        ))

        self.__generate_meta_file()
        self._application_meta = self.__create_meta_file_entry('untitled.json', 'Untitled Program', '', 0, False)

        # Setup ROS Interface
        self._update_prog_pub = rospy.Publisher('data_server/update',UpdateData, queue_size=10, latch=True)
        self._load_app_srv = rospy.Service('data_server/load_application_data',LoadData,self._load_app_cb)
        self._save_app_srv = rospy.Service('data_server/save_application_data',SaveData,self._save_app_cb)
        self._get_app_opts_srv = rospy.Service('data_server/get_application_options',GetOptions,self._get_app_opts_cb)
        self._get_prog_srv = rospy.Service('data_server/get_program',GetData,self._get_prog_cb)
        self._set_prog_srv = rospy.Service('data_server/set_program',SetData,self._set_prog_cb)
        self._get_history_srv = rospy.Service('data_server/get_history',GetHistory,self._get_history_cb)

    def spin(self):
        # give nodes time before publishing initial state
        # the topic latches message but as a best practice, it shouldn't rush
        # all other nodes with the program data until general setup is complete

        rospy.sleep(10)
        self._push_program_update()
        rospy.spin()

    #===========================================================================
    #   Application Level ROS Callbacks
    # - Load
    # - Save
    # - Get
    #===========================================================================

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
            self._program = Program.from_dct(rawData)
            self._program.changes_cb = self.__program_updated_cb
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

        self._application_meta = self.__create_meta_file_entry_from_request(request)
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

        # Try writing to file
        try:
            fout = open(os.path.join(self._application_filepath, used_filepath),'w')
            fout.write(self.__formatted_json_dump(self._program.to_dct()))
            fout.close()
        except:
            traceback.print_exc()
            response.status = False
            response.message = 'Error writing to progam file'
            return response

        # On success update meta file
        if not request.use_current_info:
            self._application_meta = self.__create_meta_file_entry_from_request(request)
            self.__append_meta_entry(self._application_meta)

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

    #===========================================================================
    #   Program Level ROS Callbacks
    # - Get
    # - Set
    # - Push Changes
    #===========================================================================

    def _get_prog_cb(self, request):
        errors = []
        response = GetDataResponse()

        if request.all:
            outData = self._program.to_dct()
        else:
            outData = {}
            inData = json.loads(request.data)
            for uuid in inData:
                try:
                    outData[uuid] = self._program_cache.get(uuid).to_dct()
                except:
                    errors.append(uuid)

        response.data = self.__formatted_json_dump(outData)
        response.tag = self._program_history.get_current_version().to_ros()
        response.status = len(errors) == 0
        response.errors = self.__formatted_json_dump(errors)
        response.message = '' if response.status else 'error getting data'
        return response

    def _set_prog_cb(self, request):
        response = SetDataResponse()

        # If the current tag matches then this is the next change to be applied
        # If not a match, then client is updating a stale version and needs to
        # refresh their local version first.
        if request.tag == self._program_history.get_current_version:
            errors = []
            inData = json.loads(request.data)
            data_snapshot = self._program.to_dct()

            if request.all:
                try:
                    self._program = Program.from_dct(inData)
                except:
                    errors.append(None)
            else:
                for change in inData:
                    uuid = change['uuid']
                    data = change['data']
                    action = change['action']

                    try:
                        if action == 'set':
                            # data is dct of node with id == uuid
                            # We need to make sure to hit the node parser for everything!
                            self._program_cache.set(uuid,data)
                        elif action == 'delete':
                            # data is the uuid of child node
                            self._program_cache.get(uuid).delete_child(data)
                        elif action == 'add':
                            # data is dct of child node
                            self._program_cache.get(uuid).add_child(data)
                        elif action == 'repair':
                            #TODO
                            pass

                    except:
                        traceback.print_exc()
                        errors.append(change)
                        break

            # If errors when attempting change then revert
            # Otherwise commit changes into history and push to clients
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
            response.errors = self.__formatted_json_dump(errors)
            response.message = '' if response.status else 'error setting data, reverted to previous program state'

        else:
            response.status = False
            response.errors = ''
            response.message = 'mismatched version tags'

        return response

    def _push_program_update(self):
        history_entry = self._program_history.get_current_entry()

        msg = UpdateData()
        msg.data = json.dumps(self._program.to_dct())
        msg.action = history_entry.action
        msg.changes = json.dumps(history_entry.changes)
        msg.currentTag = history_entry.version.to_ros()

        prev_tag = self._program_history.get_previous_version()
        msg.previousTag = prev_tag.to_ros() if prev_tag != None else Version(rospy.Duration(0),'','data-server')

        self._update_prog_pub.publish(msg)

    def __program_updated_cb(self, attribute_trace):
        pass # maybe implement a repair routine here?

    #===========================================================================
    #   History Level ROS Callbacks
    # - Get
    #===========================================================================

    def _get_history_cb(self, request):
        errors = []
        response = GetDataResponse()

        outData = None
        if request.all and not request.from_provided_tag:
            # Get entire history
            outData = self._program_history.to_dct()

        elif request.all and request.from_provided_tag:
            # Get history from tag provided to current
            try:
                version = VersionTag.from_ros(request.tag)
                entries = self._program_history.get_entries_from(version)
                outData = [e.to_dct() for e in entries]
            except:
                errors.append(request.tag.uuid)

        elif not request.all and request.from_provided_tag:
            # Get only for the tag specified
            try:
                version = VersionTag.from_ros(request.tag)
                outData = self._program_history.get_entry(version).to_dct()
            except:
                errors.append(request.tag.uuid)

        else: #not request.all and not request.from_provided_tag
            # Get current entry only
            outData = self._program_history.get_current_entry().to_dct()

        response.data = self.__formatted_json_dump(outData)
        response.tag = self._program_history.get_current_version().to_ros()
        response.status = len(errors) == 0
        response.errors = self.__formatted_json_dump(errors)
        response.message = '' if response.status else 'error getting data'
        return response
        

    #===========================================================================
    #   Meta File Utilities
    # - formatting (create methods)
    # - meta generation
    # - meta modification
    #===========================================================================

    def __create_meta_file_structure(self, task_name=''):
        return {
            'description': 'Default workspace config file for EvD',
            'name': task_name,
            'options': []
        }

    def __create_meta_file_entry(self, filename, name, description='', level=0, custom=True):
        return {
            'filename': filename,
            'name': name,
            'description': description,
            'level': level,
            'custom': custom
        }

    def __create_meta_file_entry_from_request(self, request, custom=True):
        return {
            'filename': request.filename,
            'name': request.name,
            'description': request.description,
            'level': request.level,
            'custom': custom
        }

    def __generate_meta_file(self, override_existing=False):
        try:
            if override_existing:
                raise Exception('Forcing file update')
            else: # If the file does not exist or cannot be read from
                fin = open(os.path.join(self._application_filepath,'meta.json'),'r')
                fin.close()
        except:
            fout = open(os.path.join(self._application_filepath,'meta.json'),'w')
            fout.write(self.__formatted_json_dump(self.__create_meta_file_structure('EvD Workspace')))
            fout.close()

    def __append_meta_entry(self, entry):
        # Try opening meta file
        try:
            fin = open(os.path.join(self._application_filepath,'meta.json'),'r')
            inStr = fin.read()
            fin.close()
        except:
            traceback.print_exc()
            return False, 'Error reading in meta file'

        # Try parsing file
        try:
            meta_data = json.loads(inStr)
        except:
            traceback.print_exc()
            return False, 'Error json load meta file'

        # Find if entry already exists
        idx = None
        for i in range(0,len(meta_data['options'])):
            if request.filename == meta_data['options'][i]['filename']:
                idx = i
                break

        # Update or append
        if idx != None:
            meta_data['options'][idx] = entry
        else:
            meta_data['options'].append(entry)

        # Write back new meta file
        try:
            fout = open(os.path.join(self._application_filepath,'meta.json'),'w')
            fout.write(self.__formatted_json_dump(meta_data))
            fout.close()
        except:
            traceback.print_exc()
            return False, 'Error writing to meta file'

        return True, ''

    #===========================================================================
    #   General Utilities
    # - formatted JSON dump
    #===========================================================================

    def __formatted_json_dump(self, dct, sort_keys=True):
        return json.dumps(dct, indent=4, sort_keys=sort_keys)


if __name__ == '__main__':
    rospy.init_node('data_server')

    app_filepath = rospy.get_param('~app_filepath')

    node = DataServer(app_filepath)
    node.spin()
