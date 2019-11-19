#!/usr/bin/env python

#TODO this needs to be entirely rewritten using the robot_interface

'''

Modes = [
    freedrive
    servoing
    playback-pose
    playback-trajectory
]

Robots = [
    physical
    simulated
]

'''

import json
import rospy

from std_msgs.msg import Bool

from cobots_datatypes.waypoint import Waypoint
from cobots_datatypes.trajectory import Trajectory

from cobots_core.srv import GetRobotControlMode, GetRobotControlModeRequest, GetRobotControlModeResponse
from cobots_core.srv import SetRobotControlMode, SetRobotControlModeRequest, SetRobotControlModeResponse
from cobots_core.srv import RobotControlFreedrive, RobotControlFreedriveRequest, RobotControlFreedriveResponse
from cobots_core.srv import RobotControlServoing, RobotControlServoingRequest, RobotControlServoingResponse


class RobotControlServer:

    def __init__(self):
        self._mode = 'idle'
        self._freedrive_state = False
        self._physical_servoing_state = False
        self._simulated_servoing_state = False

        self._freedrive_pub = rospy.Publisher('robot_control/freedrive',Bool,queue_size=2)
        self._phy_servo_pub = rospy.Publisher('robot_control/servoing/physical',Bool,queue_size=2)
        self._sim_servo_pub = rospy.Publisher('robot_control/servoing/simulated',Bool,queue_size=2)

        self._set_mode_srv = rospy.Service('robot_control/set_mode',SetRobotControlMode,self._set_mode_cb)
        self._get_mode_srv = rospy.Service('robot_control/get_mode',GetRobotControlMode,self._get_mode_cb)
        self._freedrive_srv = rospy.Service('robot_control/freedrive',RobotControlFreedrive,self._freedrive_cb)
        self._servoing_srv = rospy.Service('robot_control/servoing',RobotControlServoing,self._servoing_cb)

    def _set_mode_cb(self, request):
        status = True
        message = ''

        if request.mode != self._mode:
            if request.mode == SetRobotControlModeRequest.FREEDRIVE:
                self._mode = 'freedrive'
                self._freedrive_state = False
                self._physical_servoing_state = False
                self._simulated_servoing_state = False
            elif request.mode == SetRobotControlModeRequest.SERVOING:
                self._mode = 'servoing'
                self._freedrive_state = False
                self._physical_servoing_state = False
                self._simulated_servoing_state = False
            elif request.mode == SetRobotControlModeRequest.PLAYBACK:
                self._mode = 'playback'
                self._freedrive_state = False
                self._physical_servoing_state = False
                self._simulated_servoing_state = False
            elif request.mode == SetRobotControlModeRequest.IDLE:
                self._mode = 'idle'
                self._freedrive_state = False
                self._physical_servoing_state = False
                self._simulated_servoing_state = False
            else:
                status = False
                message = 'Invalid mode string'

            if status:
                self._phy_servo_pub.publish(self._physical_servoing_state)
                self._sim_servo_pub.publish(self._simulated_servoing_state)
                self._freedrive_pub.publish(self._freedrive_state)

        response = SetRobotControlModeResponse()
        response.status = status
        response.message = message
        return response

    def _get_mode_cb(self, request):
        response = GetRobotControlModeResponse()
        response.mode = self._mode
        return response

    def _freedrive_cb(self, request):
        status = True
        message = ''

        if self._mode != 'freedrive':
            status = False
            message = 'not in freedrive mode'
        else:
            self._freedrive_state = request.state
            self._freedrive_pub.publish(self._freedrive_state)

        response = RobotControlFreedriveResponse()
        response.status = status
        response.message = message
        return response

    def _move_to_waypoint_cb(self, goal):
        pass #TODO define action

    def _trace_trajectory_cb(self, goal):
        pass #TODO define action

    def _servoing_cb(self, request):
        status = True
        message = ''

        if self._mode != 'servoing':
            status = False
            message = 'not in servoing mode'
        else:
            self._physical_servoing_state = request.physical
            self._phy_servo_pub.publish(self._physical_servoing_state)
            self._simulated_servoing_state = request.simulated
            self._sim_servo_pub.publish(self._simulated_servoing_state)

        response = RobotControlServoingResponse()
        response.status = status
        response.message = message
        return response


if __name__ == '__main__':
    rospy.init_node('robot_control_server')

    node = RobotControlServer()

    rospy.spin()
