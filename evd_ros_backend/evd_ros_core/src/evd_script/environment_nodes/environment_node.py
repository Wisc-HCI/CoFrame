'''
EnvironmentNode is a namespace wrapper to group all environment data-types together
'''

from ..node import Node


class EnvironmentNode(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'environment-node' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, type='', name='', parent=None, uuid=None, append_type=True,
                 editable=True, deleteable=True, description=''):
        super(EnvironmentNode,self).__init__(
            type=EnvironmentNode.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)
