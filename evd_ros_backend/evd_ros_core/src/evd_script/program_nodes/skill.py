'''
Skills act as functions within EvD. Like Hierarchical blocks they provide a higher
level definition to a collection of nodes. However, they also support 
parameterization, which allows them to be reused as if they are primitives.

To invoke a skill in EvD, there is a primitive control_flow block called CallSkill
which interfaces with this node to fill in the parameterization 
'''

from .primitive import Primitive


class Skill(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim):
        return 'skill' + ('.' if trailing_delim else '')
    
    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True, 
                 editable=True, deleteable=True, description=''):

        super(Skill,self).__init__(
            type=Skill.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

    def to_dct(self):
        msg = super(Skill,self).to_dct()
        msg.update({

        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            uuid=dct['uuid']
        )

    '''
    Data accessor/modifier methods
    '''


