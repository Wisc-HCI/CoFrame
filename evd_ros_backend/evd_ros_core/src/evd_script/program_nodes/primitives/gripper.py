'''
Gripper exposes the robot's grasping behavior in a generalized fashion.
Position, speed, and effort paramters ought to be supplied. Internal kinematics
of the gripper is handled at the implementation level.

#TODO implement real-time behavior
'''

from ..primitive import Primitive


class Gripper(Primitive):

    '''
    Constants
    '''

    SEMANTIC_AMBIGUOUS = 'ambiguous'
    SEMANTIC_GRASPING = 'grasping'
    SEMANTIC_RELEASING = 'releasing'

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'gripper' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, position=0, effort=0, speed=0, thing_uuid=None, semantic=None,
                 type='', name='', uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description=''):

        self._thing_uuid = None
        self._position = None
        self._effort = None
        self._speed = None
        self._semantic = None

        super(Gripper,self).__init__(
            type=Gripper.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.position = position
        self.effort = effort
        self.speed = speed
        self.thing_uuid = thing_uuid

        if thing_uuid != None:
            self.semantic = semantic if semantic != None else self.SEMANTIC_GRASPING

    def to_dct(self):
        msg = super(Gripper,self).to_dct()
        msg.update({
            'thing_uuid': self.thing_uuid,
            'position': self.position,
            'effort': self.effort,
            'speed': self.speed,
            'semantic': self.semantic
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            position=dct['position'],
            effort=dct['effort'],
            speed=dct['speed'],
            thing_uuid=dct['thing_uuid'],
            semantic=dct['semantic'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        if self._position != value:
            self._position = value
            self.updated_attribute('position','set')

    @property
    def effort(self):
        return self._effort

    @effort.setter
    def effort(self, value):
        if self._effort != value:
            self._effort = value
            self.updated_attribute('effort','set')

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, value):
        if self._speed != value:
            self._speed = value
            self.updated_attribute('speed','set')

    @property
    def thing_uuid(self):
        return self._thing_uuid

    @thing_uuid.setter
    def thing_uuid(self, value):
        if self._thing_uuid != value:
            self._thing_uuid = value

            if self._thing_uuid == None:
                # without a reference to a thing, behavior is unknowable
                self.semantic = self.SEMANTIC_AMBIGUOUS
            elif self.semantic == self.SEMANTIC_AMBIGUOUS:
                # assume attempting to grasp by default
                self.semantic = self.SEMANTIC_GRASPING

            self.updated_attribute('thing_uuid','set')

    @property
    def semantic(self):
        return self._semantic

    @semantic.setter
    def semantic(self, value):
        if self._semantic != value:

            if self.thing_uuid == None:
                if value != self.SEMANTIC_AMBIGUOUS:
                    raise Exception('If no thing is defined then semantic can only be ambiguous')
            else:
                if value != self.SEMANTIC_GRASPING and value != self.SEMANTIC_RELEASING:
                    raise Exception('If thing defined then it must either be grasping or releasing')

            self._semantic = value
            self.updated_attribute('semantic','set')

    def set(self, dct):
        position = dct.get('position', None)
        if position != None:
            self.position = position

        effort = dct.get('effort', None)
        if effort != None:
            self.effort = effort

        speed = dct.get('speed', None)
        if speed != None:
            self.speed = speed

        if 'thing_uuid' in dct.keys():
            self.thing_uuid = dct['thing_uuid']

        if 'semantic' in dct.keys():
            self.semantic = dct['semantic']

        super(Gripper,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):

        super(Gripper,self).deep_update()

        self.updated_attribute('thing_uuid','update')
        self.updated_attribute('position','update')
        self.updated_attribute('effort','update')
        self.updated_attribute('speed','update')
        self.updated_attribute('semantic','update')

    def shallow_update(self):
        super(Gripper,self).shallow_update()

        self.updated_attribute('thing_uuid','update')
        self.updated_attribute('position','update')
        self.updated_attribute('effort','update')
        self.updated_attribute('speed','update')
        self.updated_attribute('semantic','update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        hooks.tokens['robot']['state']['gripper']['position'] = self.position

        if self.thing_uuid != None:
            if self.semantic == self.SEMANTIC_GRASPING:
                hooks.tokens['robot']['state']['gripper']['grasped_thing'] = self.thing_uuid
            elif self.semantic == self.SEMANTIC_RELEASING:
                hooks.tokens['robot']['state']['gripper']['grasped_thing'] = None
        else:
            hooks.tokens['robot']['state']['gripper']['grasped_thing'] = None
            hooks.tokens['robot']['state']['gripper']['ambiguous_flag'] = True

        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        next = self

        if not self.uuid in hooks.state.keys():
            hooks.state[self.uuid] = 'pending'
            hooks.robot_interface.grip_async(self.position, self.speed, self.effort)

        else:
            resp = hooks.robot_interface.is_acked('gripper')
            if resp != None:
                if resp:
                    del hooks.state[self.uuid]
                    next = self.parent

                    if self.thing_uuid != None:
                        if self.semantic == self.SEMANTIC_GRASPING:
                            hooks.tokens['robot']['state']['gripper']['grasped_thing'] = self.thing_uuid
                        elif self.semantic == self.SEMANTIC_RELEASING:
                            hooks.tokens['robot']['state']['gripper']['grasped_thing'] = None
                    else:
                        hooks.tokens['robot']['state']['gripper']['grasped_thing'] = None
                        hooks.tokens['robot']['state']['gripper']['ambiguous_flag'] = True

                else:
                    raise Exception('Robot NACKed')

        status = hooks.robot_interface.get_status()
        hooks.tokens['robot']['state']['gripper']['position'] = status.gripper_position

        return next
