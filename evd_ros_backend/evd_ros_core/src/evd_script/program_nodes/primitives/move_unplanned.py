'''
Jogs the robot to a location without pre-planning a trajectory within EvD.

THIS IS AN UNSAFE PRIMITIVE! Only use within the context of initialization.

TODO implement thing token movement behavior
'''

import numpy as np

from ..primitive import Primitive
from ... import BOOLEAN_TYPE, ENUM_TYPE, NUMBER_TYPE
from ...data_nodes.location import Location
from ...data_nodes.geometry.pose import Pose
from ...data_nodes.geometry.position import Position
from ...data_nodes.geometry.orientation import Orientation


class MoveUnplanned(Primitive):

    TYPES = ['joint', 'ee_ik']

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Move Unplanned'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'move-unplanned' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Primitive.template()
        template['parameters'].append({
            'type': BOOLEAN_TYPE,
            'key': 'manual_safety',
            'is_uuid': False,
            'is_list': False
        })
        template['parameters'].append({
            'type': NUMBER_TYPE,
            'key': 'velocity',
            'is_uuid': False,
            'is_list': False
        })
        template['parameters'].append({
            'type': ENUM_TYPE,
            'key': 'move_type',
            'is_uuid': False,
            'is_list': False,
            'enum_values': [x for x in cls.TYPES]
        })
        template['parameters'].append({
            'type': Location.full_type_string(),
            'key': 'location_uuid',
            'is_uuid': True,
            'is_list': False        
        })
        return template

    def __init__(self, locUuid, manual_safety=True, move_type="joint", velocity=0,
                 type='', name='', uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description='', parameters=None):

        if parameters == None:
            parameters = {
                'velocity': velocity,
                'move_type': move_type,
                'location_uuid': locUuid,
                'manual_safety': manual_safety
            }

        super(MoveUnplanned,self).__init__(
            type=MoveUnplanned.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters)

    '''
    Data accessor/modifier methods
    '''

    @property
    def velocity(self):
        return self._parameters['velocity']

    @velocity.setter
    def velocity(self, value):
        if self._parameters['velocity'] != value:
            self._parameters['velocity'] = value
            self.updated_attribute('parameters.velocity','set')

    @property
    def move_type(self):
        return self._parameters['move_type']

    @move_type.setter
    def move_type(self, value):
        if self._parameters['move_type'] != value:

            if not value in self.TYPES:
                raise Exception("Invalid move_type provided")

            self._parameters['move_type'] = value
            self.updated_attribute('parameters.move_type','set')

    @property
    def manual_safety(self):
        return self._parameters['manual_safety']

    @manual_safety.setter
    def manual_safety(self, value):
        if self._parameters['manual_safety'] != value:
            self._parameters['manual_safety'] = value
            self.updated_attribute('parameters.manual_safety','set')

    @property
    def location_uuid(self):
        return self._parameters['location_uuid']

    @location_uuid.setter
    def location_uuid(self, value):
        if self._parameters['location_uuid'] != value:
            self._parameters['location_uuid'] = value
            self.updated_attribute('parameters.location_uuid','set')

    def set(self, dct):
        if 'location_uuid' in dct.keys():
            self.location_uuid = dct['location_uuid']

        if 'manual_safety' in dct.keys():
            self.manual_safety = dct['manual_safety']

        velocity = dct.get('velocity',None)
        if velocity != None:
            self.velocity = velocity

        move_type = dct.get('move_type',None)
        if move_type != None:
            self.move_type = move_type

        super(MoveUnplanned,self).set(dct)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        # compute initial thing state
        Ttr = self._handle_initial_thing_state(hooks)
        hooks.state[self.uuid] = {'status': 'pending', 'Ttr': Ttr}

        # update robot state
        loc = self.context.get_location(self.location_uuid)
        hooks.tokens['robot']['state']['position'] = loc.position.to_simple_dct()
        hooks.tokens['robot']['state']['orientation'] = loc.orientation.to_simple_dct()
        hooks.tokens['robot']['state']['joints'] = loc.joints

        # update thing state
        self._handle_current_thing_state(hooks)

        del hooks.state[self.uuid]
        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        next = self

        thing_uuid = hooks.tokens['robot']['state']['gripper']['grasped_thing']

        if not self.uuid in hooks.state.keys():
            # Set initial state and start action
            hooks.robot_interface.is_acked('arm') # clear prev ack
            Ttr = self._handle_initial_thing_state(hooks)
            hooks.state[self.uuid] = {'status': 'pending', 'Ttr': Ttr}
            loc = self.context.get_location(self.location_uuid)
            hooks.robot_interface.move_async(loc, self.move_type, self.velocity, self.manual_safety)

        else:
            resp = hooks.robot_interface.is_acked('arm')
            if resp != None:
                if resp:
                    next = self.parent

                else:
                    raise Exception('Robot NACKed')

        # update current state
        self._handle_current_robot_state(hooks)
        self._handle_current_thing_state(hooks)

        if next == self.parent:
            del hooks.state[self.uuid]
        return next

    def _handle_initial_thing_state(self, hooks):
        thing_uuid = hooks.tokens['robot']['state']['gripper']['grasped_thing']

        # Compute transform from thing to robot
        Ttr = None
        if None != thing_uuid:
            Ttr, _ = Pose.compute_relative(
                Pose.from_simple_dct(hooks.tokens[thing_uuid]['state']),
                Pose.from_simple_dct(hooks.tokens['robot']['state']))

        return Ttr

    def _handle_current_robot_state(self,hooks):
        status = hooks.robot_interface.get_status()
        hooks.tokens['robot']['state']['position'] = Position.from_ros(status.arm_pose.position).to_simple_dct()
        hooks.tokens['robot']['state']['orientation'] = Orientation.from_ros(status.arm_pose.orientation).to_simple_dct()
        hooks.tokens['robot']['state']['joints'] = status.arm_joints

    def _handle_current_thing_state(self, hooks):
        thing_uuid = hooks.tokens['robot']['state']['gripper']['grasped_thing']

        #Set thing state / Compute transform thing to world
        if None != thing_uuid:
            Trw = Pose.from_simple_dct(hooks.tokens['robot']['state']).to_matrix()
            Ttr = hooks.state[self.uuid]['Ttr']
            Ttw = Pose.matrix_inverse(np.matmul(Ttr,Pose.matrix_inverse(Trw)))
            new_thing_pose = Pose.from_matrix(Ttw).to_simple_dct()
            hooks.tokens[thing_uuid]['state'].update(new_thing_pose)
