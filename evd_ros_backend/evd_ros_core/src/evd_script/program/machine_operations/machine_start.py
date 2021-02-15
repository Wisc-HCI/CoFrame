from .machine_primitive import MachinePrimitive


class MachineStart(MachinePrimitive):

    '''
    Data structure methods
    '''

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineStart,self).__init__(
            machineUuid=machineUuid,
            type='machine-start.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.machine_start(self)

    def realtime_execution(self, hooks):
        hooks.machine_start(self)
