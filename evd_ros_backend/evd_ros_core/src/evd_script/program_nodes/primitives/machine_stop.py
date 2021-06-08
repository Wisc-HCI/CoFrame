'''
Commands machine to stop running a routine. Actual implemenation subject
to application engineer.

This might not do anything if the process is already done (and that is fine).
Of course stopping a process early might be problematic so care should be taken
when implementing this in an application.
'''

from ..machine_primitive import MachinePrimitive


class MachineStop(MachinePrimitive):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Machine Stop'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-stop' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return MachinePrimitive.full_type_string() + cls.type_string()

    def __init__(self, machineUuid=None, parameters=None, type='', name='', uuid=None, parent=None,
                 append_type=True, editable=True, deleteable=True, description=''):
        super(MachineStop,self).__init__(
            machineUuid=machineUuid,
            type=MachineStop.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters)

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self
        hooks.tokens[self.machine_uuid]['state'] = 'idle'
        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        next = self

        if not self.uuid in hooks.state.keys():
            hooks.machine_interface.is_acked(self.machine_uuid) # clear prev ack
            hooks.tokens[self.machine_uuid]['state'] = 'pending'
            hooks.machine_inferface.stop(self.machine_uuid)
            hooks.state[self.uuid] = 'pending'
        else:
            resp = hooks.machine_interface.is_acked(self.machine_uuid)
            if resp != None:
                if resp:
                    hooks.tokens[self.machine_uuid]['state'] = 'idle'
                    del hooks.state[self.uuid]
                    next = self.parent
                else:
                    raise Exception('Machine NACKed - {}'.format(self.machine_uuid))

        return next