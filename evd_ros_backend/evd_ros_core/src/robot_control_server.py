#!/usr/bin/env python3

'''
High-level control of robot instance subsystems.
'''

import rospy

from std_msgs.msg import Empty, Bool
from evd_interfaces.robot_interface import RobotInterface
from evd_interfaces.data_client_interface import DataClientInterface
from evd_interfaces.program_runner import ProgramRunner


class RobotControlServer:

    def __init__(self):
        self._playing = False
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
                    self._start_freedrive()

                elif action == 'stop_freedrive' and not self._playing:
                    self._stop_freedrive()

                elif action == 'play':
                    if self._program.pause:
                        self._program.pause = False
                    else:
                        physical = self._physical_interface if self._use_physical_robot else None
                        simulated = self._simulated_interface if self._use_simulated_robot else None
                        self._program = ProgramRunner(self._data_interface.program, physical, simulated)
                        self._program.start()

                elif action == 'stop' and self._playing:
                    self._program.stop()
                    self._playing = False

                elif action == 'pause' and self._playing:
                    self._program.pause = True

                elif action == 'reset':
                    self._reset_state()

            # Update underlying program state on robot
            if self._playing:
                self._program.update()
            rate.sleep()

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
