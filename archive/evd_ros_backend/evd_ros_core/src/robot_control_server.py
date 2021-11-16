#!/usr/bin/env python3

'''
High-level control of robot and machines for simulation
'''

import json
import rospy

from evd_script import NodeParser, Program
from std_msgs.msg import Empty, Bool, String
from evd_ros_core.msg import ProgramRunnerStatus
from evd_ros_core.srv import SetData, SetDataResponse

from evd_interfaces.program_runner import ProgramRunner
from evd_interfaces.robot_interface import RobotInterface
from evd_interfaces.machine_interface import MachineInterface


class RobotControlServer:

    def __init__(self):
        self._playing = False
        self._cmd_queue = []
        self._program = None
        self._player = None

        # External Interfaces
        self._machine_interface = MachineInterface()
        self._robot_interface = RobotInterface()

        # Robot Controls
        self._play_sub = rospy.Subscriber('robot_control_server/play',Empty,self._play_cb)
        self._stop_sub = rospy.Subscriber('robot_control_server/stop',Empty,self._stop_cb)
        self._pause_sub = rospy.Subscriber('robot_control_server/pause',Empty,self._pause_cb)
        self._reset_sub = rospy.Subscriber('robot_control_server/reset',Empty,self._reset_cb)
        
        # Program execution point
        self.set_program_srv = rospy.Service('robot_control_server/set_program',SetData,self._set_program_cb)

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

    def _set_program_cb(self, request):
        response = SetDataResponse()

        if self._playing:
            response.status = False
            response.message = 'Currently playing program - stop first!'
        else:
            old_prog = self._program
            try:
                raw = json.loads(request.data)
                self._program = NodeParser(raw, enforce_types=[Program.type_string()])

                response.status = True
                response.message = ''
            except:
                response.status = False
                response.message = 'Failed to deserialize program - reverting'
                self._program = old_prog
            
        return response

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
                    if self._player != None:

                        # If program is currently paused then resume
                        # If program is constructed and valid then start (unknown start state, will reset)
                        if self._player.pause:
                            self._playing = True
                            self._player.pause = False
                        else:
                            self._playing = True
                            self._player.start()

                    # If could not run with existing program (or there is none)
                    # then construct a new program runner
                    if not self._playing:

                        # Create program runner and start running if all good
                        # otherwise we need to stop and report an error
                        if self._program != None:
                            self._player = ProgramRunner(
                                full_program=self._program,
                                root_node=self._program,
                                symbolic=False,
                                robot=self._robot_interface,
                                machine=self._machine_interface,
                                player=self)
                            
                            self._playing = True
                            self._player.start()
                        
                        else:
                           self.set_error('root_node_uuid not valid')

                elif action == 'stop' and self._playing:
                    self._playing = False
                    self._player.stop()
                
                elif action == 'stop' and not self._playing:
                    self.set_error('program is not running')

                elif action == 'pause' and self._playing:
                    self._player.pause = True

                elif action == 'pause' and not self._playing:
                    self.set_error('program is not running')

                elif action == 'reset':
                    if self._playing:
                        self._playing = False
                        self._player.stop()
                    
                    self._player.reset()

            # Update underlying program state on robot
            if self._playing:
                self._player.update()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('robot_control_server')
    node = RobotControlServer()
    node.spin()
