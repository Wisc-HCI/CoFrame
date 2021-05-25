'''
Commands machine to start running a routine. Actual implemenation subject
to application engineer.
'''

from .machine_primitive import MachinePrimitive


class MachineStart(MachinePrimitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-start' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return MachinePrimitive.full_type_string() + cls.type_string()

    def __init__(self, machineUuid=None, type='', name='', uuid=None, parent=None, append_type=True):
        super(MachineStart,self).__init__(
            machineUuid=machineUuid,
            type=MachineStart.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self
        hooks.tokens[self.machine_uuid]['state'] = 'running'
        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        next = self

        if not self.uuid in hooks.state.keys():
            hooks.machine_interface.is_acked(self.machine_uuid) # clear prev ack
            hooks.tokens[self.machine_uuid]['state'] = 'pending'
            hooks.machine_inferface.start(self.machine_uuid)
            hooks.state[self.uuid] = 'pending'
        else:
            resp = hooks.machine_interface.is_acked(self.machine_uuid)
            if resp != None:
                if resp:
                    hooks.tokens[self.machine_uuid]['state'] = 'running'
                    del hooks.state[self.uuid]
                    next = self.parent
                else:
                    raise Exception('Machine NACKed'.format(self.machine_uuid))

        return next
