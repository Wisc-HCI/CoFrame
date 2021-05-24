'''
Commands machine to stop running a routine. Actual implemenation subject
to application engineer.

This might not do anything if the process is already done (and that is fine).
Of course stopping a process early might be problematic so care should be taken
when implementing this in an application.
'''

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
