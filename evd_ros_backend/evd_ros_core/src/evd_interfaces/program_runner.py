'''
Program runner takes in a raw program (can be a full program or a subset of nodes)
and will attempt to run it using a standard player interface.

The program runner also acts as its own set of hooks when passed into the EvDscript
AST for execution. It allows the underlying program to command machines and robot.
It allows breakpoints to pause execution. And it provides both a state scratchpad for
nodes and a token tracker for persistent state across nodes.

Each executable node should implement symbolic and realtime execution methods. 

Real-time execution runs a simulation that actually runs through the execution pipeline.
Whereas the symbolic execution merely manipulate the symbols needed to achieve 
post-conditions from pre-conditions.
'''

import time

from evd_ros_core.msg import ProgramRunnerStatus


class ProgramRunner(object):

    #===========================================================================
    # Program Runner External Interface
    #===========================================================================

    def __init__(self, full_program, root_node, symbolic=False, robot=None, machine=None, player=None):
        self._symbolic = symbolic
        self._program = full_program
        self._root_node = root_node
        self._robot_interface = robot
        self._machine_interface = machine
        self._player_interface = player

        self._state = {} # nodes can place internal state here, indexed by their node UUID
        self._tokens = {} # tokens are currently things, machine state
        self._pause = False
        self._active_node = None # hooked into by the nodes themselves (really just a flagging mechanism)
        self._next_node = None # node that drives the execution state machine

        self._start_time = -1
        self._prev_time = -1
        self._curr_time = -1
        self._stop_time = -1

    @property
    def root_uuid(self):
        return self._root_node.uuid

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

        # initialize state
        error = self.reset()
        
        # Give initial step of program execution
        if not error:
            self.update()
            if not self._symbolic:
                self._player_interface.set_at_start(False)

        return error

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
        self._next_node = self._root_node 
        self._state = {}

        # fill in tokens (with unknown states)
        self._tokens = { 'robot': {
            'type': 'robot', 'state': {
                'position': {'x':'?','y':'?','z':'?'}, 
                'orientation': {'x':'?','y':'?','z':'?','w':'?'},
                'joints': ['?'],
                'gripper': {'position': '?', 'grasped_thing': None, 'ambiguous_flag': False}
            }
        }}
        for e in self._program.context.machines:
            self._tokens[e.uuid] = {'type': 'machine', 'state': '?'}
        for e in self._program.context.things:
            self._tokens[e.uuid] = {'type': 'thing', 'state': {'position': e.position.to_dct(), 'orientation': e.orientation.to_dct()}}

        # generate reasonable start state given an arbitrary node (must symbolically execute up to root_node)
        # result will be a token table with up-to-date values
        error = False
        next_node = self._program
        while next_node != self._root_node and not error:
            next_node = next_node.symbolic_execution(self)
            error = next_node == None

        if not self._symbolic:
            self._player_interface.set_at_start(True)
            self._player_interface.set_at_end(False)
            
            if error:
                self._player_interface.set_error('Reset could not move state to root node')
        
        return error

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

                # When root node is not the root program, we must stop subtree execution when 
                # it tries to execute the parent above the root node.
                if self._next_node != None and self._next_node == self._root_node.parent:
                    self._next_node = None # on the next update, we will find that we ended the program

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

    @property
    def program(self):
        return self._program