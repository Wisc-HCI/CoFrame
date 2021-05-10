from ..primitive import Primitive


class Gripper(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls):
        return 'gripper.'

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, position=0, effort=0, speed=0, thing_uuid=None, type='',
                 name='', uuid=None, parent=None, append_type=True):

        self._thing_uuid = None
        self._position = None
        self._effort = None
        self._speed = None

        super(Gripper,self).__init__(
            type='gripper.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.thing_uuid = thing_uuid
        self.position = position
        self.effort = effort
        self.speed = speed

    def to_dct(self):
        msg = super(Gripper,self).to_dct()
        msg.update({
            'thing_uuid': self.thing_uuid,
            'position': self.position,
            'effort': self.effort,
            'speed': self.speed
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            position=dct['position'],
            effort=dct['effort'],
            speed=dct['speed'],
            thing_uuid=dct['thing_uuid'])

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
            self.updated_attribute('thing_uuid','set')

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

    def shallow_update(self):
        super(Gripper,self).shallow_update()

        self.updated_attribute('thing_uuid','update')
        self.updated_attribute('position','update')
        self.updated_attribute('effort','update')
        self.updated_attribute('speed','update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        pass

    def realtime_execution(self, hooks):
        pass
