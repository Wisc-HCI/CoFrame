from .machine_primitive import MachinePrimitive


class MachineStop(MachinePrimitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-stop' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return MachinePrimitive.full_type_string() + cls.type_string()

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineStop,self).__init__(
            machineUuid=machineUuid,
            type=MachineStop.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        pass

    def realtime_execution(self, hooks):
        pass
