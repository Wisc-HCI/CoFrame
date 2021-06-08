'''
Skills act as functions within EvD. Like Hierarchical blocks they provide a higher
level definition to a collection of nodes. However, they also support 
parameterization, which allows them to be reused as if they are primitives.

To invoke a skill in EvD, there is a primitive control_flow block called SkillCall
which interfaces with this node to fill in the parameterization 

Nesting SkillCalls is a bit tricky. The crux of the problem is that a Skill / Skill
Arguement requires top-level parameterization but a skill-call provides this as a
set of "shadow-params".
#TODO think about how best to do this linking, maybe all primitives have a parameters list?
'''

from ..data_nodes.skill_arguement import SkillArguement
from .hierarchical import Hierarchical
from ..node_parser import NodeParser
from .. import ALL_PRIMITIVES_TYPES


class Skill(Hierarchical):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Skill'

    @classmethod
    def type_string(cls, trailing_delim):
        return 'skill' + ('.' if trailing_delim else '')
    
    @classmethod
    def full_type_string(cls):
        return Hierarchical.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Hierarchical.template()
        template['fields'].append({
            'type': SkillArguement.full_type_string(),
            'key': 'arguements',
            'is_uuid': False,
            'is_list': True
        })
        return template

    def __init__(self, arguements=None, primitives=[], type='', name='', uuid=None, 
                 parent=None, append_type=True, editable=True, deleteable=True, 
                 description='', parameters=None):

        self._arguements = {}

        super(Skill,self).__init__(
            primitives=primitives,
            type=Skill.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters)

        self.arguements = arguements if arguements == None else []

    def to_dct(self):
        msg = super(Skill,self).to_dct()
        msg.update({
            'arguements': [a.to_dct() for a in self.arguements]
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
            primitives=[NodeParser(p, enforce_types=[ALL_PRIMITIVES_TYPES]) for p in dct['primitives']],
            arguements=[NodeParser(a, enforce_types=[SkillArguement.type_string(trailing_delim=False)]) for a in dct['arguements']])

    '''
    Data accessor/modifier methods
    '''
    
    @property
    def arguements(self):
        return self._arguements.values()

    @arguements.setter
    def arguements(self, value):
        if value == None:
            raise Exception('Arguements cannot be none, must at least be an empty list')
        
        if self._arguements != None:
            for a in self._arguements.values():
                a.remove_from_cache()

        self._arguements = {}

        for a in value:
            self._arguements[a.uuid] = a
            a.parent = self

        self.updated_attribute('arguements','set')

    @property
    def arguements_dct(self):
        return self._arguements

    def add_skill_arguement(self, arg):
        arg.parent = self
        self._arguements[arg.uuid] = arg
        self.updated_attribute('arguements','add',arg.uuid)

    def delete_skill_arguement(self, uuid):
        if uuid in self._arguements.keys():
            self._arguements[uuid].remove_from_cache()
            del self._arguements[uuid]
            self.updated_attribute('arguements','delete',uuid)

    def get_skill_arguement(self, uuid):
        for a in self.arguements:
            if a.uuid == uuid:
                return a
        return None

    def add_primitive(self, prm, arg_uuids=None):
        if arg_uuids != None:
            self.apply_args_to_primitive(prm,arg_uuids)

        super(Skill,self).add_primitive(prm)

    def insert_primitive(self, idx, prm, arg_uuids=None):
        if arg_uuids != None:
            self.apply_args_to_primitive(prm,arg_uuids)

        super(Skill,self).insert_primitive(idx,prm)

    def set(self, dct):

        if 'arguements' in dct.keys():
            self.arguements=[NodeParser(a, enforce_types=[SkillArguement.type_string(trailing_delim=False)]) for a in dct['arguements']]

        super(Skill,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for a in self.arguements:
            a.remove_from_cache()

        super(Skill,self).remove_from_cache()

    def add_to_cache(self):
        for a in self.arguements:
            a.add_to_cache()

        super(Skill,self).add_to_cache()

    '''
    Children methods
    '''

    def delete_child(self, uuid):
        success = True

        if uuid in [a.uuid for a in self.arguements]:
            self.delete_skill_arguement(uuid)
        else:
            success = False

        if not success: 
            success = super(Skill,self).delete_child(uuid)

        return success

    '''
    Update Methods
    '''

    def late_construct_update(self):
        
        for a in self.arguements:
            a.late_construct_update()

        super(Skill,self).late_construct_update()

    def deep_update(self):
        
        for a in self.arguements:
            a.deep_update()

        super(Skill,self).deep_update()

        self.updated_attribute('arguements','update')

    def shallow_update(self):
        super(Skill,self).shallow_update()

        self.updated_attribute('arguements','update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        raise Exception('A skill does not execute directly, it must be invoked by a skill-call, which turns it into a hierarchical block')

    def realtime_execution(self, hooks):
        raise Exception('A skill does not execute directly, it must be invoked by a skill-call, which turns it into a hierarchical block')

    '''
    Utility methods
    '''
    
    def apply_args_to_primitive(self, prm, arg_uuids):
        # arg_uuids = [
        #   '<skill-arguement-uuid>
        # ]

        for uuid in arg_uuids:
                if uuid not in self._arguements.keys():
                    raise Exception('Skill arguement is being used before it exists in the arguements list')

                key = self._arguements[uuid].parameter_key
                value = self._arguements[uuid].temporary_value

                getattr(prm,key) = value

    def resolve_to_hierarchical(self, arg_map):
        # arg_map = {
        #    '<skill-arg-uuid>': <actual_value>,
        #    ...
        # }
        
        unmapped_args = self._arguements.keys()
        primitives_copy = [p.to_dct() for p in self.primitives]

        for arg_uuid in arg_map.keys():
            if arg_uuid in self._arguements.keys():
                unmapped_args.remove(arg_uuid)

                value = arg_map[arg_uuid]
                key = self._arguements[arg_uuid].parametr_key
                temp = self._arguements[arg_uuid].temporary_value

                for p in primitives_copy:
                    if key in p.keys() and p[key] == temp:
                        p[key] = value

        if len(unmapped_args) > 0:
            return None # could not resolve down to hierarchical node
        else:
            return Hierarchical.from_dct({
                'primitives': primitives_copy,
                'name': self.name,
                'type': Hierarchical.full_type_string(),
                'append_type': False,
                'editable': False,
                'deleteable': False,
                'description': '(Autogenerated hierarchical from skill) :: ' + self.description,
                'uuid': self._generate_uuid(Hierarchical.full_type_string() + '-autogenerated-from-' + self.uuid)})
