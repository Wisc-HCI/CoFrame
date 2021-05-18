from ..primitive import Primitive
from ...node_parser import NodeParser


class Branch(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'branch' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, type='', name='', uuid=None,
                 parent=None, append_type=True):

        super(Branch,self).__init__(
            type=Branch.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    def to_dct(self):
        pass

    @classmethod
    def from_dct(self, value):
        pass

    '''
    Data accessor/modifier methods
    '''

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for e in self.entries:
            e.condition.remove_from_cache()
            e.primitive.remove_from_cache()

        super(Branch,self).remove_from_cache()

    def add_to_cache(self):
        for e in self.entries:
            e.condition.add_to_cache()
            e.primitive.add_to_cache()

        super(Branch,self).add_to_cache()

    '''
    Update Methods
    '''

    def late_construct_update(self):

        for e in self.entries:
            e.condition.late_construct_update()
            e.primitive.late_construct_update()

        super(Branch,self).late_construct_update()

    def deep_update(self):

        for e in self.entries:
            e.condition.deep_update()
            e.primitive.deep_update()

        super(Branch,self).deep_update()

        self.updated_attribute('entries','update')

    def shallow_update(self):
        super(Branch,self).shallow_update()

        self.updated_attribute('entries','update')
