#!/usr/bin/env python

'''

Modes = [
    freedrive
    servoing
    pose
    trajectory
]

Robots = [
    physical
    simulated
]

'''

import json
import rospy

from robot_interface import RobotInterface


class RobotControlServer:

    def __init__(self):
        self._simulated_interface = RobotInterface('simulated')
        self._physical_interface = RobotInterface('physical')


if __name__ == '__main__':
    rospy.init_node('robot_control_server')

    node = RobotControlServer()

    rospy.spin()
