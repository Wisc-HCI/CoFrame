'''
Jogs the robot to a location without pre-planning a trajectory within EvD.

THIS IS AN UNSAFE PRIMITIVE! Only use within the context of initialization.

TODO implement thing token movement behavior
'''

from ..primitive import Primitive
from ...data.geometry.position import Position
from ...data.geometry.orientation import Orientation


class MoveUnplanned(Primitive):

    TYPES = ['joint', 'ee_ik']

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'move-unplanned' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, locUuid, manual_safety=True, move_type="joint", velocity=0,
                 type='', name='', uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description=''):

        self._velocity = None
        self._move_type = None
        self._location_uuid = None
        self._manual_safety = None

        super(MoveUnplanned,self).__init__(
            type=MoveUnplanned.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.velocity = velocity
        self.move_type = move_type
        self.manual_safety = manual_safety
        self.location_uuid = locUuid

    def to_dct(self):
        msg = super(MoveUnplanned,self).to_dct()
        msg.update({
            'location_uuid': self.location_uuid,
            'manual_safety': self.manual_safety,
            'velocity': self.velocity,
            'move_type': self.move_type
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            uuid=dct['uuid'],
            locUuid=dct['location_uuid'],
            manual_safety=dct['manual_safety'],
            velocity=dct['velocity'],
            move_type=dct['move_type'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if self._velocity != value:
            self._velocity = value
            self.trace = None
            self.updated_attribute('velocity','set')

    @property
    def move_type(self):
        return self._move_type

    @move_type.setter
    def move_type(self, value):
        if self._move_type != value:

            if not value in self.TYPES:
                raise Exception("Invalid move_type provided")

            self._move_type = value
            self.trace = None
            self.updated_attribute('move_type','set')

    @property
    def manual_safety(self):
        return self._manual_safety

    @manual_safety.setter
    def manual_safety(self, value):
        if self._manual_safety != value:
            self._manual_safety = value
            self.updated_attribute('manual_safety','set')

    @property
    def location_uuid(self):
        return self._location_uuid

    @location_uuid.setter
    def location_uuid(self, value):
        if self._location_uuid != value:
            self._location_uuid = value
            self.updated_attribute('location_uuid','set')

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
    Update Methods
    '''

    def deep_update(self):

        super(MoveUnplanned,self).deep_update()

        self.updated_attribute('velocity','update')
        self.updated_attribute('move_type','update')
        self.updated_attribute('location_uuid','update')
        self.updated_attribute('manual_safety','update')

    def shallow_update(self):
        super(MoveUnplanned,self).shallow_update()

        self.updated_attribute('velocity','update')
        self.updated_attribute('move_type','update')
        self.updated_attribute('location_uuid','update')
        self.updated_attribute('manual_safety','update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        loc = self.context.get_location(self.location_uuid)
        hooks.tokens['robot']['state']['position'] = loc.position.to_simple_dct()
        hooks.tokens['robot']['state']['orientation'] = loc.orientation.to_simple_dct()
        hooks.tokens['robot']['state']['joints'] = loc.joints

        #TODO handle thing movement

        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        next = self

        if not self.uuid in hooks.state.keys():
            hooks.robot_interface.is_acked('arm') # clear prev ack
            hooks.state[self.uuid] = 'pending'
            loc = self.context.get_location(self.location_uuid)
            hooks.robot_interface.move_async(loc, self.move_type, self.velocity, self.manual_safety)

        else:
            resp = hooks.robot_interface.is_acked('arm')
            if resp != None:
                if resp:
                    del hooks.state[self.uuid]
                    next = self.parent

                else:
                    raise Exception('Robot NACKed')

        status = hooks.robot_interface.get_status()
        hooks.tokens['robot']['state']['position'] = Position.from_ros(status.arm_pose.position).to_simple_dct()
        hooks.tokens['robot']['state']['orientation'] = Orientation.from_ros(status.arm_pose.orientation).to_simple_dct()
        hooks.tokens['robot']['state']['joints'] = status.arm_joints

        #TODO handle thing movement

        return next

        return self.symbolic_execution(hooks)
