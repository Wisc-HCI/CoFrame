from ..primitive import Primitive


class Breakpoint(Primitive):

    '''
    Data structure methods
    '''

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(Breakpoint,self).__init__(
            type='breakpoint.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)
