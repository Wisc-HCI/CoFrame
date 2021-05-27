#!/usr/bin/env python3

'''
High-level control of robot and machines for simulation
'''

import json
import rospy

from std_msgs.msg import Empty, Bool, String
from evd_ros_core.msg import ProgramRunnerStatus
from evd_ros_core.srv import SetRootNode, SetRootNodeResponse
from evd_ros_core.srv import GetRootNode, GetRootNodeResponse

from evd_interfaces.program_runner import ProgramRunner
from evd_interfaces.robot_interface import RobotInterface
from evd_interfaces.machine_interface import MachineInterface
from evd_interfaces.data_client_interface import DataClientInterface


class RobotControlServer:

    def __init__(self):
        self._root_node_uuid = ''
        self._playing = False
        self._cmd_queue = []
        self._program = None

        # Data server interface
        self._data_interface = DataClientInterface(use_application_interface=False)

        # External Interfaces
        self._machine_interface = MachineInterface()
        self._robot_interface = RobotInterface()

        # Robot Controls
        self._play_sub = rospy.Subscriber('robot_control_server/play',Empty,self._play_cb)
        self._stop_sub = rospy.Subscriber('robot_control_server/stop',Empty,self._stop_cb)
        self._pause_sub = rospy.Subscriber('robot_control_server/pause',Empty,self._pause_cb)
        self._reset_sub = rospy.Subscriber('robot_control_server/reset',Empty,self._reset_cb)
        
        # Program execution point
        self.set_root_node_srv = rospy.Service('robot_control_server/set_root_node',SetRootNode,self._set_root_node_cb)
        self.get_root_node_srv = rospy.Service('robot_control_server/get_root_node',GetRootNode,self._get_root_node_cb)

        # Player Feedback
        self._at_start_pub = rospy.Publisher('robot_control_server/at_start',Bool, queue_size=10, latch=True)
        self._at_end_pub = rospy.Publisher('robot_control_server/at_end',Bool, queue_size=10, latch=True)
        self._lockout_pub = rospy.Publisher('robot_control_server/lockout',Bool, queue_size=10, latch=True)
        self._status_pub = rospy.Publisher('robot_control_server/status',ProgramRunnerStatus, queue_size=10)
        self._tokens_pub = rospy.Publisher('robot_control_server/tokens',String, queue_size=10)
        self._errors_pub = rospy.Publisher('robot_control_server/error',String,queue_size=10)

    def _play_cb(self, _):
        self._cmd_queue.append('play')

    def _stop_cb(self, _):
        self._cmd_queue.append('stop')

    def _pause_cb(self, _):
        self._cmd_queue.append('pause')

    def _reset_cb(self, _):
        self._cmd_queue.append('reset')

    def _set_root_node_cb(self, request):
        response = SetRootNodeResponse()

        if self._playing:
            response.status = False
            response.message = 'Currently playing program - stop first!'
        else:   

            if request.uuid != self._root_node_uuid:
                self._root_node_uuid = request.uuid
                self._program = None
            
            response.status = True
            response.message = ''

        return response

    def _get_root_node_cb(self, _):
        return GetRootNodeResponse(self._root_node_uuid)

    def set_at_start(self,state):
        self._at_start_pub.publish(Bool(state))
    
    def set_at_end(self,state):
        self._at_end_pub.publish(Bool(state))

    def set_lockout(self,state):
        self._lockout_pub.publish(Bool(state))

    def set_status(self,msg):
        self._status_pub.publish(msg)

    def set_tokens(self,dct):
        self._tokens_pub.publish(String(json.dumps(dct)))

    def set_error(self,str_msg):
        self._errors_pub.publish(String(str_msg))

    def spin(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            # Process command
            if len(self._cmd_queue) > 0:
                action = self._cmd_queue.pop(0)

                if action == 'play':
                    self._playing = False

                    # Try running with existing program
                    if self._program != None:

                        # If program is currently paused then resume
                        # If program is constructed and valid then start (unknown start state, will reset)
                        if self._program.pause:
                            self._playing = True
                            self._program.pause = False
                            
                        elif self._program.root_uuid == self._root_node_uuid:
                            self._playing = True
                            self._program.start()

                    # If could not run with existing program (or there is none)
                    # then construct a new program runner
                    if not self._playing:

                        # Find root level node to start executing
                        uuid = self._root_node_uuid
                        if uuid == '' or uuid == None:
                            node = self._data_interface.program
                        else: 
                            try:
                                node = self._data_interface.cache.get(uuid)
                            except:
                                node = None

                        # Create program runner and start running if all good
                        # otherwise we need to stop and report an error
                        if node != None:
                            self._program = ProgramRunner(
                                full_program=self._data_interface.program,
                                root_node=node,
                                symbolic=False,
                                robot=self._robot_interface,
                                machine=self._machine_interface,
                                player=self)
                            
                            self._playing = True
                            self._program.start()
                        
                        else:
                           self.set_error('root_node_uuid not valid')

                elif action == 'stop' and self._playing:
                    self._playing = False
                    self._program.stop()
                
                elif action == 'stop' and not self._playing:
                    self.set_error('program is not running')

                elif action == 'pause' and self._playing:
                    self._program.pause = True

                elif action == 'pause' and not self._playing:
                    self.set_error('program is not running')

                elif action == 'reset':
                    if self._playing:
                        self._playing = False
                        self._program.stop()
                    
                    self._program.reset()

            # Update underlying program state on robot
            if self._playing:
                self._program.update()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('robot_control_server')
    node = RobotControlServer()
    node.spin()
