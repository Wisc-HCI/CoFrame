from .machine_primitive import MachinePrimitive


class MachineInitialize(MachinePrimitive):

    '''
    Data structure methods
    '''

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None,
                 append_type=True):
        super(MachineInitialize,self).__init__(
            machineUuid=machineUuid,
            type='machine-initialize.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.machine_initialize(self)

    def realtime_execution(self, hooks):
        hooks.machine_initialize(self)
