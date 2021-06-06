'''

NOTE: skills need some way of checking their associated skill from program

NOTE: needs to shadow their configured arguements as properties, 
such that they can be chained inside of skills

the key of the shadow_param must be the name of a skill_arg in the skill
referenced by skill_uuid. Value will be expanded based on type information
in the skill-arg
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
        template['fields'].append({
            'type': Skill.full_type_string(),
            'key': 'skill_uuid',
            'is_uuid': True,
            'is_list': False
        })
        template['fields'].append({
            'type': ARBITRARY_OBJ_TYPE, # dict with keys of parameter name and values of desired value
            'key': 'shadow_params',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, skill_uuid=None, shadow_params=None, type='', name='', uuid=None, parent=None, 
                 append_type=True, editable=True, deleteable=True, description=''):

        self._shadow_params = None

        super(SkillCall,self).__init__(
            type=SkillCall.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self._skill_uuid = skill_uuid
        self.updated_attribute('skill_uuid','set')

        self.shadow_params = shadow_params if shadow_params != None else {}
        
    def to_dct(self):
        msg = super(SkillCall,self).to_dct()
        msg.update({
            'skill_uuid': self.skill_uuid,
            'shadow_params': self.shadow_params
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
            uuid=dct['uuid'],
            skill_uuid=dct['skill_uuid'],
            shadow_params=dct['shadow_params'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def skill_uuid(self):
        return self._skill_uuid

    @property
    def shadow_params(self):
        return self._shadow_params

    @shadow_params.setter
    def shadow_params(self, value):
        if self._shadow_params != value:
            if value == None:
                raise Exception('Shadow params must be a valid dictionary')

            self._shadow_params = value
            self.updated_attribute('shadow_params','set')

            for param_name in self._shadow_params.keys():
                self.updated_attribute(param_name,'set')

    def add_shadow_param(self, name, value):
        self._shadow_params[name] = value
        self.updated_attribute('shadow_params','add',name)
        self.updated_attribute(name,'set')

    def delete_shadow_param(self, name):
        del self._shadow_params[name]

        self.updated_attribute('shadow_params','delete',name)
        self.updated_attribute(name,'delete')

    def __getattr__(self, name):
        #TODO this overrides the rest of properties which need to be exposed
        if name in self._shadow_params.keys():
            return self._shadow_params[name]
        else:
            raise AttributeError()

    def __setattr__(self, name, value):
        #TODO this overrides the rest of properties which need to be exposed
        if name == 'skill_uuid':
            raise AttributeError()

        self.add_shadow_param(name,value)

    def set(self, dct):
        # cannot set skill_uuid

        if 'shadow_params' in dct.keys():
            self.shadow_params = dct['shadow_params']

        super(SkillCall,self).set(dct)

    '''
    Update methods
    '''

    def deep_update(self):
        super(SkillCall,self).deep_update()

        self.updated_attribute('skill_uuid','update')
        self.updated_attribute('shadow_params','update')

        for param_name in self.shadow_params.keys():
            self.updated_attribute(param_name,'update')

    def shallow_update(self):
        super(SkillCall,self).shallow_update()

        self.updated_attribute('skill_uuid','update')
        self.updated_attribute('shadow_params','update')

        for param_name in self.shadow_params.keys():
            self.updated_attribute(param_name,'update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        hooks.active_primitive = self

        next = None

        if not self.uuid in hooks.state.keys():
            skill = hooks.program.skills[self.skill_uuid]

            #self.shadow_params -> keys are names of parameters, values are actual values
            #skill.arguements_dct -> dict of SkillArguements (name is shadow_param key, we need uuid, is key)
            
            arg_map = {}
            for shadow_param_name, shadow_param_value in self.shadow_params.items():
                for arg_uuid, arg in skill.arguements_dct.items():
                    if arg.name == shadow_param_name:
                        arg_map[arg_uuid] = shadow_param_value
            
            hierarchicalBlock = skill.resolve_to_hierarchical(arg_map)
            if hierarchicalBlock == None:
                raise Exception('Unable to resolve hierarchical block from skill')
            
            hierarchicalBlock.parent = self
            hooks.state[self.uuid] = { 'block': hierarchicalBlock }

            next = hierarchicalBlock
        else:
            hooks.state[self.uuid]['block'].remove_from_cache()
            del hooks.state[self.uuid]
            next = self.parent

        return next

    def realtime_execution(self, hooks):
        return self.symbolic_execution(hooks)