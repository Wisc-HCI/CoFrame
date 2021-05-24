'''
Commands a machine to be initialized. Actual implemenation is up to the 
application engineer.
'''

from .machine_primitive import MachinePrimitive


class MachineInitialize(MachinePrimitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-initialize' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return MachinePrimitive.full_type_string() + cls.type_string()

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None,
                 append_type=True):
        super(MachineInitialize,self).__init__(
            machineUuid=machineUuid,
            type=MachineInitialize.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self
        hooks.tokens[self.machine_uuid]['state'] = 'initialized'
        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        next = self

        if not self.uuid in hooks.state.keys():
            hooks.tokens[self.machine_uuid]['state'] = 'pending'
            hooks.machine_initialize(self.machine_uuid)
            hooks.state[self.uuid] = 'pending'
        else:
            if hooks.machine_get_status(self.machine_uuid) == 'done':
                hooks.tokens[self.machine_uuid]['state'] = 'initialized'
                del hooks.state[self.uuid]
                next = self.parent

        return next
