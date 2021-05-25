'''
Program runner takes in a raw program (can be a full program or a subset of nodes)
and will attempt to run it using a standard player interface.

The program runner also acts as its own set of hooks when passed into the EvDscript
AST for execution. It allows the underlying program to command machines and robot.
It allows breakpoints to pause execution. And it provides both a state scratchpad for
nodes and a token tracker for persistent state across nodes.

Each executable node should implement symbolic and realtime execution methods. 
'''

import time

from evd_ros_core.msg import ProgramRunnerStatus


class ProgramRunner(object):

    #===========================================================================
    # Program Runner External Interface
    #===========================================================================

    def __init__(self, raw_program, symbolic=False, robot=None, machine=None, player=None):
        self._symbolic = symbolic
        self._program = raw_program
        self._robot_interface = robot
        self._machine_interface = machine
        self._player_interface = player

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
    def root_uuid(self):
        return self._program.uuid

    @property
    def pause(self):
        return self._pause

    @pause.setter
    def pause(self, value):
        if self._pause != value:
            self._pause = value

            # Send pause behavior to robots
            self._robot_interface.pause(self._pause)

            # Send pause signal to all machines
            for machine in self._program.context.machines:
                self._machine_interface.pause(self._pause, machine.uuid)

    def start(self):
        if not self._symbolic:
            self._player_interface.set_lockout(True)

        self.reset() # initialize state
        self.update()
        if not self._symbolic:
            self._player_interface.set_at_start(False)

    def stop(self):
        self._stop_time = time.time()
        self._start_time = -1
        self._prev_time = -1
        self._curr_time = -1

        # Cancel active pending robot actions
        self._robot_interface.estop()

        # Send stop signal to all machines
        for machine in self._program.context.machines:
            self._machine_interface.estop(machine.uuid)

        # Send stop and unlock messages
        if not self._symbolic:
            self._player_interface.set_at_end(True)
            self._player_interface.set_lockout(False)

    def reset(self):
        self._prev_time = -1
        self._curr_time = time.time()
        self._start_time = self._curr_time
        self._stop_time = -1

        self._active_node = None
        self._next_node = self._program
        self._state = {}

        # fill in tokens (with unknown states)
        # TODO generate reasonable start state given an arbitrary node
        self._tokens = { 'robot': {'type': 'robot', 'state': {'position': {'x':'?','y':'?','z':'?'}, 'orientation': {'x':'?','y':'?','z':'?','w':'?'}}} }
        for e in self._program.context.machines:
            self._tokens[e.uuid] = {'type': 'machine', 'state': '?'}
        for e in self._program.context.things:
            self._tokens[e.uuid] = {'type': 'thing', 'state': {'position': e.position.to_dct(), 'orientation': e.orientation.to_dct()}}

        if not self._symbolic:
            self._player_interface.set_at_start(True)
            self._player_interface.set_at_end(False)

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
                    self._player_interface.set_at_end(True)
                    self._player_interface.set_lockout(False)

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
            
            self._player_interface.set_status(msg)

    def _publish_tokens(self):
        if not self._symbolic:
            self._player_interface.set_tokens(self._tokens)

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

    @property
    def machine_interface(self):
        return self._machine_interface

    @property
    def robot_interface(self):
        return self._robot_interface