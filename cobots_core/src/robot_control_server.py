#!/usr/bin/env python

import json
import rospy

from std_msgs.msg import Empty, Bool
from interfaces.robot_interface import RobotInterface
from interfaces.data_client_interface import DataClientInterface
from interfaces.program_runner import ProgramRunner

class RobotControlServer:

    def __init__(self):
        self._paused = False
        self._playing = False
        self._program = None
        self._cmd_queue = []
        self._use_physical_robot = False
        self._use_simulated_robot = False

        # Data server interface
        self._data_interface = DataClientInterface()

        # Lower-level robot interfaces
        self._simulated_interface = RobotInterface('simulated')
        self._physical_interface = RobotInterface('physical')

        # Robot Controls
        self._use_simulated_robot_sub = rospy.Subscriber('robot_control_server/use_simulated_robot',Bool,self._use_simulated_robot_cb)
        self._use_physical_robot_sub = rospy.Subscriber('robot_control_server/use_physical_robot',Bool,self._use_physical_robot_cb)
        self._freedrive_sub = rospy.Subscriber('robot_control_server/freedrive',Bool,self._freedrive_cb)
        self._play_sub = rospy.Subscriber('robot_control_server/play',Empty,self._play_cb)
        self._stop_sub = rospy.Subscriber('robot_control_server/stop',Empty,self._stop_cb)
        self._pause_sub = rospy.Subscriber('robot_control_server/pause',Empty,self._pause_cb)
        self._reset_sub = rospy.Subscriber('robot_control_server/reset',Empty,self._reset_cb)
        self._step_fwd_sub = rospy.Subscriber('robot_control_server/step_forward',Empty,self._step_fwd_cb)
        self._step_bkd_sub = rospy.Subscriber('robot_control_server/step_backward',Empty,self._step_bkd_cb)

    def _use_simulated_robot_cb(self, msg):
        self._use_simulated_robot = msg.data

    def _use_physical_robot_cb(self, msg):
        self._use_physical_robot = msg.data

    def _freedrive_cb(self, msg):
        if msg.data:
            self._cmd_queue.append('start_freedrive')
        else:
            self._cmd_queue.append('stop_freedrive')

    def _play_cb(self, noop):
        self._cmd_queue.append('play')

    def _stop_cb(self, noop):
        self._cmd_queue.append('stop')

    def _pause_cb(self, noop):
        self._cmd_queue.append('pause')

    def _reset_cb(self, noop):
        self._cmd_queue.append('reset')

    def _step_fwd_cb(self, noop):
        self._cmd_queue.append('step_forward')

    def _step_bkd_cb(self, noop):
        self._cmd_queue.append('step_backward')

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Process command
            if len(self._cmd_queue) > 0:
                action = self._cmd_queue.pop(0)

                if action == 'start_freedrive':
                    if self._playing:
                        self._program.stop()
                        self._playing = False
                        self._paused = False
                    self._start_freedrive()

                elif action == 'stop_freedrive' and not self._playing:
                    self._stop_freedrive()

                elif action == 'play':
                    if self._paused:
                        self._program.unpause()
                        self._paused = False

                    else:
                        robots = []
                        if self._use_physical_robot:
                            robots.append(self._physical_interface)
                        if self._use_simulated_robot:
                            robots.append(self._simulated_interface)
                        self._program = ProgramRunner(self._data_interface.get_runnable_program(), robots)
                        self._program.start()

                elif action == 'stop':
                    if self._playing:
                        self._program.stop()
                        self._playing = False
                        self._paused = False

                elif action == 'pause' and self._playing:
                    self._paused = True
                    self.program.pause()

                elif action == 'reset':
                    self._reset_state()

                elif action == 'step_forward' and self._playing:
                    self.program.step_forward()

                elif action == 'step_backward' and self._playing:
                    self.program.step_backward()

            # Update underlying program state on robot
            if self._playing:
                self.program.update()
            rate.spin()

    def _start_freedrive(self):
        self._simulated_interface.freedrive_pub.publish(True)
        self._physical_interface.freedrive_pub.publish(True)

    def _stop_freedrive(self):
        self._simulated_interface.freedrive_pub.publish(False)
        self._physical_interface.freedrive_pub.publish(False)


if __name__ == '__main__':
    rospy.init_node('robot_control_server')
    node = RobotControlServer()
    node.spin()
