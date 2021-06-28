'''
Stalls robot's execution until the machine signals that it has completed its
process.

This behavior is required on implemetation of a machine lest programs assume
a static timing (bad programming).

#TODO handle thing behavior (at end, transformers and producers resolve placeholders into things)
'''

from ..machine_primitive import MachinePrimitive


class MachineWait(MachinePrimitive):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Machine Wait'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'machine-wait' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return MachinePrimitive.full_type_string() + cls.type_string()

    def __init__(self, machine_uuid=None, parameters=None, type='', name='', uuid=None, parent=None,
                 append_type=True, editable=True, deleteable=True, description=''):
        super(MachineWait,self).__init__(
            machine_uuid=machine_uuid,
            type=MachineWait.type_string() + type if append_type else type,
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

        status = hooks.machine_interface.get_status(self.machine_uuid)
        if not status['running']:
            if status['status'] == 'idle':
                next = self.parent
            elif status['status'] == 'error':
                raise Exception('Machine status in error, failed waiting')

        return next
