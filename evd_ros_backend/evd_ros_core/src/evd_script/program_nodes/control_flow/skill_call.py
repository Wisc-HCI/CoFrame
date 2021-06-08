'''
Skill Call invokes a particular skill defined by its uuid. A skill call 
will generate a concrete hierarchical block to execute at run-time.
'''

from ... import ARBITRARY_OBJ_TYPE
from ..primitive import Primitive
from ..skill import Skill


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
        template['parameters'].append({
            'type': Skill.full_type_string(),
            'key': 'skill_uuid',
            'is_uuid': True,
            'is_list': False
        })
        return template

    def __init__(self, skill_uuid=None, parameters=None, type='', name='', uuid=None, parent=None, 
                 append_type=True, editable=True, deleteable=True, description=''):

        if parameters == None:
            parameters = {
                'skill_uuid': skill_uuid
            }

        super(SkillCall,self).__init__(
            type=SkillCall.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters)

    '''
    Data accessor/modifier methods
    '''

    @property
    def skill_uuid(self):
        return self._skill_uuid

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        next = None

        if not self.uuid in hooks.state.keys():
            skill = hooks.program.skills_dct[self.skill_uuid]

            #self.parameters -> keys are names of parameters, values are actual values
            #skill.arguements_dct -> dict of SkillArguements (name is parameters key, we need uuid, is key)
            
            arg_map = {}
            for param_name, param_value in self.parameters.items():
                for arg_uuid, arg in skill.arguements_dct.items():
                    if arg.name == param_name:
                        arg_map[arg_uuid] = param_value
            
            hierarchicalBlock = skill.resolve_to_hierarchical(arg_map)
            if hierarchicalBlock == None:
                raise Exception('Unable to resolve hierarchical block from skill')
            
            hierarchicalBlock.parent = self
            hooks.state[self.uuid] = { 'block': hierarchicalBlock }

            next = hierarchicalBlock
        else:
            hooks.state[self.uuid]['block'].remove_from_cache()
            next = self.parent

        if next == self.parent:
            del hooks.state[self.uuid]
        return next

    def realtime_execution(self, hooks):
        return self.symbolic_execution(hooks)