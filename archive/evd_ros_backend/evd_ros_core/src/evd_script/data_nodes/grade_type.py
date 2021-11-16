'''

'''
from ..node import Node


class GradeType(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Grade Type'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'grade-type' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description=''):
        super(GradeType,self).__init__(
            type=GradeType.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)
