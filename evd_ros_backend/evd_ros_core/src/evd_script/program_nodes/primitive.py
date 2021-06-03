'''
Primitive is a sub-type of node that indicates it can be executed in a program.
'''

from ..node import Node


class Primitive(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'primitive' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description=''):
        super(Primitive,self).__init__(
            type=Primitive.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)
