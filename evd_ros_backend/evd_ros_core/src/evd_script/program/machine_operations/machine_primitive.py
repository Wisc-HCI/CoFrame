'''
Machine Primitive extends primitive to simplify the later definition of specific machine primitives.

A machine primitive is a way to generalize machine behavior in EvD. The actual behavior needs to be
supplied externally and is hooked into the EvD program runner.
'''

from ..primitive import Primitive


class MachinePrimitive(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-primitive' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True, editable=True, deleteable=True):
        self._machine_uuid = None

        super(MachinePrimitive,self).__init__(
            type=MachinePrimitive.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable)

        self.machine_uuid = machineUuid

    def to_dct(self):
        msg = super(MachinePrimitive,self).to_dct()
        msg.update({
            'machine_uuid': self.machine_uuid
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            machineUuid=dct['machine_uuid'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def machine_uuid(self):
        return self._machine_uuid

    @machine_uuid.setter
    def machine_uuid(self, value):
        if self._machine_uuid != value:
            self._machine_uuid = value
            self.updated_attribute('machine_uuid','set')

    def set(self, dct):
        if 'machine_uuid' in dct.keys():
            self.machine_uuid = dct['machine_uuid']

        super(MachinePrimitive,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(MachinePrimitive,self).deep_update()

        self.updated_attribute('machine_uuid','update')

    def shallow_update(self):
        super(MachinePrimitive,self).shallow_update()

        self.updated_attribute('machine_uuid','update')
