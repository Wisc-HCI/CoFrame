from ..primitive import Primitive


class Breakpoint(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'breakpoint' + '.' if trailing_delim else ''

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(Breakpoint,self).__init__(
            type=Breakpoint.type_string() + type if append_type else type,
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
