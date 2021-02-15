from ..primitive import Primitive


class MoveUnplanned(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls):
        return 'move-unplanned.'

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, locUuid, manual_safety=True, type='', name='', uuid=None,
                 parent=None, append_type=True):

        self._location_uuid = None
        self._manual_safety = None

        super(MoveUnplanned,self).__init__(
            type='move-unplanned.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.manual_safety = manual_safety
        self.location_uuid = locUuid

    def to_dct(self):
        msg = super(MoveUnplanned,self).to_dct()
        msg.update({
            'location_uuid': self.location_uuid,
            'manual_safety': self.manual_safety
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'],
            locUuid=dct['location_uuid'],
            manual_safety=dct['manual_safety'])

    '''
    Data accessor/modifier methods
    '''

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

        super(MoveUnplanned,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):

        super(MoveUnplanned,self).deep_update()

        self.updated_attribute('location_uuid','update')
        self.updated_attribute('manual_safety','update')

    def shallow_update(self):
        super(MoveUnplanned,self).shallow_update()

        self.updated_attribute('location_uuid','update')
        self.updated_attribute('manual_safety','update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.move_unplanned(self)

    def realtime_execution(self, hooks):
        hooks.move_unplanned(self)
