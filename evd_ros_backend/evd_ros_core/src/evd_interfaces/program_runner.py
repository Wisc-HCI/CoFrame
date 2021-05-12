'''
???
'''

import time
import json
import rospy

from std_msgs.msg import Bool, String
from evd_ros_core.msg import ProgramRunnerStatus


class ProgramRunner(object):

    #===========================================================================
    # Program Runner External Interface
    #===========================================================================

    def __init__(self, raw_program, physical_robot_interface=None, simulated_robot_interface=None,
                 physical_machine_interface=None, simulated_machine_interface=None, symbolic=False):
        self._symbolic = symbolic
        self._program = raw_program
        self._physical_robot_interface = physical_robot_interface
        self._simulated_robot_interface = simulated_robot_interface
        self._physical_machine_interface = physical_machine_interface
        self._simulated_machine_interface = simulated_machine_interface

        mode = None
        if not self._symbolic:
            if physical_robot_interface and simulated_robot_interface:
                mode = 'both'
                if not physical_machine_interface or not simulated_machine_interface:
                    raise Exception('Both machine interfaces must be provided if both robot interfaces used')
            elif physical_robot_interface:
                mode = 'physical'
                if not physical_machine_interface:
                    raise Exception('Physical machine interface must be supplied')
            elif simulated_robot_interface:
                mode = 'simulated'
                if not simulated_machine_interface:
                    raise Exception('Simulated machine interface must be supplied')
            else:
                raise Exception('Must be at least one valid robot interface')
        self._mode = mode

        if not self._symbolic:
            self._at_start_pub = rospy.Publisher('program_runner/at_start',Bool, queue_size=10, latch=True)
            self._at_end_pub = rospy.Publisher('program_runner/at_end',Bool, queue_size=10, latch=True)
            self._lockout_pub = rospy.Publisher('program_runner/lockout',Bool, queue_size=10, latch=True)
            self._status_pub = rospy.Publisher('program_runner/status',ProgramRunnerStatus, queue_size=10, latch=True)
            self._tokens_pub = rospy.Publisher('program_runner/tokens',String, queue_size=10, latch=True)

        self._state = {} # nodes can place internal state here, indexed by their node UUID
        self._tokens = {} # tokens are currently things, machine state
        self._pause = False
        self._active_node = None
        self._next_node = None

        self._start_time = -1
        self._prev_time = -1
        self._curr_time = -1
        self._stop_time = -1

    @property
    def mode(self):
        return self._mode

    @property
    def pause(self):
        return self._pause

    @pause.setter
    def pause(self, value):
        if self._pause != value:
            self._pause = value

            # Send pause behavior to robots
            # Send pause signal to all machines
            if self._physical_robot_interface:
                self._physical_robot_interface.pause(self._pause)
                for machine in self._program.environment.machines:
                    self._physical_machine_interface.pause(self._pause, machine.uuid)

            if self._simulated_robot_interface:
                self._simulated_robot_interface.pause(self._pause)
                for machine in self._program.environment.machines:
                    self._simulated_machine_interface.pause(self._pause, machine.uuid)

    def start(self):
        self.reset() # initialize state
        if not self._symbolic:
            self._at_start_pub.publish(Bool(True))
            self._lockout_pub.publish(Bool(True))

        self.update()
        if not self._symbolic:
            self._at_start_pub.publish(Bool(False))

    def stop(self):
        self._stop_time = time.time()
        self._start_time = -1
        self._prev_time = -1
        self._curr_time = -1

        # Send stop and unlock messages
        if not self._symbolic:
            self._at_end_pub.publish(True)
            self._lockout_pub.publish(Bool(False))

        # Cancel active pending robot actions
        # Send stop signal to all machines
        if self._physical_robot_interface:
            self._physical_robot_interface.estop()
            for machine in self._program.environment.machines:
                self._physical_machine_interface.estop(machine.uuid)

        if self._simulated_robot_interface:
            self._simulated_robot_interface.estop()
            for machine in self._program.environment.machines:
                self._simulated_machine_interface.estop(machine.uuid)

    def reset(self):
        self._prev_time = -1
        self._curr_time = time.time()
        self._start_time = self._curr_time
        self._stop_time = -1

        self._active_node = None
        self._next_node = self._program
        self._state = {}

        # fill in tokens
        self._tokens = { 'robot': {'type': 'robot', 'state': {'position': {'x':'?','y':'?','z':'?'}, 'orientation': {'x':'?','y':'?','z':'?','w':'?'}}} }
        for e in self._program.environment.machines:
            self._tokens[e.uuid] = {'type': 'machine', 'state': '?'}
        for e in self._program.environment.things:
            self._tokens[e.uuid] = {'type': 'thing', 'state': {'position': e.position.to_dct(), 'orientation': e.orientation.to_dct()}}

        self._at_end_pub.publish(False)

    def update(self):
        self._prev_time = self._curr_time
        self._curr_time = time.time()

        hasMore = True
        if not self._pause:
            self._publish_tokens()

            if self._next_node:
                # While there is still program nodes to run
                if self._symbolic:
                    self._next_node = self._next_node.symbolic_execution(self)
                else:
                    self._next_node = self._next_node.realtime_execution(self)
            else:
                # At Program End
                if not self._symbolic:
                    self._at_end_pub.publish(True)
                    self._lockout_pub.publish(Bool(False))

                self._stop_time = time.time()
                self._publish_status()
                hasMore = False

        return hasMore

    def _publish_status(self):
        if not self._symbolic:
            msg = ProgramRunnerStatus()
            msg.uuid = self._active_node.uuid if self._active_node else ''
            msg.start_time = self._start_time
            msg.previous_time = self._prev_time
            msg.current_time = self._curr_time
            msg.stop_time = self._stop_time
            self._status_pub.publish(msg)

    def _publish_tokens(self):
        if not self._symbolic:
            msg = String()
            msg.data = json.dumps(self._tokens)
            self._tokens_pub.publish(msg)

    #===========================================================================
    # Program Runner Hooks
    #===========================================================================

    @property
    def active_node(self):
        return self._active_node

    @active_node.setter
    def active_node(self, value):
        if self._active_node != value:
            self._active_node = value
            self._publish_status()

    @property
    def state(self):
        return self._state

    @property
    def tokens(self):
        return self._tokens

    @property
    def previous_time(self):
        return self._prev_time

    @property
    def current_time(self):
        return self._curr_time

    @property
    def start_time(self):
        return self._start_time

    #===========================================================================
    # Machine Hooks
    #===========================================================================

    def machine_initialize(self, uuid):
        if self._physical_machine_interface:
            self._physical_machine_interface.machine_initialize(uuid)
        if self._simulated_machine_interface:
            self._simulated_machine_interface.machine_initialize(uuid)

    def machine_start(self, uuid):
        if self._physical_machine_interface:
            self._physical_machine_interface.machine_start(uuid)
        if self._simulated_machine_interface:
            self._simulated_machine_interface.machine_start(uuid)

    def machine_stop(self, uuid):
        if self._physical_machine_interface:
            self._physical_machine_interface.machine_stop(uuid)
        if self._simulated_machine_interface:
            self._simulated_machine_interface.machine_stop(uuid)

    def machine_get_status(self, uuid):
        status = None
        if self._physical_machine_interface:
            status = self._physical_machine_interface.get_status(uuid)
        else:
            status = self._simulated_machine_interface.get_status(uuid)
        return status

    #===========================================================================
    # Robot Hooks
    #===========================================================================
