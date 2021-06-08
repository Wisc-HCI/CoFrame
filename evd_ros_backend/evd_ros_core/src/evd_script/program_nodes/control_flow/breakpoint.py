'''
Breakpoint exposes pause behavior in the runner to the program.

This is useful for debugging code.
'''

from ..primitive import Primitive


class Breakpoint(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Breakpoint'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'breakpoint' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description='', parameters=None):
        super(Breakpoint,self).__init__(
            type=Breakpoint.type_string() + type if append_type else type,
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
        # breakpoints do nothing in symbolic mode
        hooks.active_primitive = self
        return self.parent

    def realtime_execution(self, hooks):
        hooks.active_primitive = self
        hooks.pause = True
        return self.parent
