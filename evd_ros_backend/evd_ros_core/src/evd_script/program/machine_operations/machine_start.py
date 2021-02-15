from .machine_primitive import MachinePrimitive


class MachineStart(MachinePrimitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls):
        return 'machine-start.'

    @classmethod
    def full_type_string(cls):
        return MachinePrimitive.full_type_string() + cls.type_string()

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineStart,self).__init__(
            machineUuid=machineUuid,
            type='machine-start.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)
