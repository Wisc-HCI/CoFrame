'''

NOTE: needs to shadow their configured arguements as propertiess
'''

from ..primitive import Primitive


class SkillCall(Primitive):
    
    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Skill Call'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'skill-call' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Primitive.template()
        template['fields'].append({
            
        })
        return template


    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description=''):
        super(SkillCall,self).__init__(
            type=SkillCall.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

    def to_dct(self):
        msg = super(SkillCall,self).to_dct()
        msg.update({

        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            
        )

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        return self.parent #TODO

    def realtime_execution(self, hooks):
        return self.parent #TODO