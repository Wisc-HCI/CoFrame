from ..node import Node


class Primitive(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls):
        return 'primitive.'

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        super(Primitive,self).__init__(
            type='primitive.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)
