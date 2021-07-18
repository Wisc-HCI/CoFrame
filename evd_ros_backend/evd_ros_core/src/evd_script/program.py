'''
Program is a top-level hierarchical task that wraps execution of robot behavior and provides
hook to global Environment/Context. Additionally, all change traces end at this
root node in the AST. Users can hook into this with a change callback.

Program also provides a list of skills defined as "macros/functions" to be invoked within
the AST. Simple parameterization is provided.
'''

from .node_parser import NodeParser
from .environment import Environment
from .program_nodes.hierarchical import Hierarchical
from .program_nodes import skills_library
from .type_defs import ALL_PRIMITIVES_TYPES

from .orphans import *


class Program(Hierarchical):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Program'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'program' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Hierarchical.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Hierarchical.template()
        template['fields'].append({
            'type': Environment.full_type_string(),
            'key': 'environment',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': Skill.full_type_string(),
            'key': 'skills',
            'is_uuid': False,
            'is_list': True
        })
        return template

    def __init__(self, primitives=[], skills=None, environment=None, changes_cb=None, 
                 issues_cb=None, name='', type='', uuid=None, append_type=True, 
                 editable=True, deleteable=True, description=''):
        self._orphan_list = evd_orphan_list()
        self.changes_cb = changes_cb
        self.issues_cb = issues_cb
        self._environment = None
        self._skills = None

        if environment == None:
            environment = Environment()

        if not isinstance(environment,Environment):
            raise Exception('Program level context must be an environment')

        super(Program,self).__init__(
            type=Program.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=None,
            append_type=append_type,
            primitives=primitives,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.environment = environment
        if skills != None:
            self.skills = skills
        else: 
            self.skills = [s() for s in skills_library]

    def to_dct(self):
        msg = super(Program,self).to_dct()
        msg.update({
            'environment': self.environment.to_dct(),
            'skills': [s.to_dct() for s in self.skills]
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'],
            primitives=[NodeParser(p, enforce_types=[ALL_PRIMITIVES_TYPES]) for p in dct['primitives']],
            environment=NodeParser(dct['environment'], enforce_types=[Environment.type_string(trailing_delim=False)]),
            skills=[NodeParser(s, enforce_types=[Skill.type_string(trailing_delim=False)]) for s in dct['skills']])

    '''
    Data accessor/modifier methods
    '''

    @property
    def context(self):  # Alias
        return self.environment

    @context.setter
    def context(self, value):   # Alias
        self.environment = value

    @property
    def environment(self):
        return self._environment

    @environment.setter
    def environment(self, value):
        if self._environment != value:
            if not isinstance(value,Environment):
                raise Exception('Program level context must be an environment')

            if self._environment != None:
                self._environment.remove_from_cache()

            self._environment = value
            self._environment.parent = self

            self.updated_attribute('context','set')
            self.updated_attribute('environment','set')

    @property
    def skills(self):
        return self._skills.values()

    @skills.setter
    def skills(self, value):
        if value == None:
            raise Exception('Skills must be a list of skill nodes')

        if self._skills != None:
            for s in self._skills.values():
                s.remove_from_cache()
        
        self._skills = {}
        for s in value:
            self._skills[s.uuid] = s
            s.parent = self

        self.updated_attribute('skills','set')


    @property
    def skills_dct(self):
        return self._skills

    def add_skill(self, skill):
        skill.parent = self
        self._skills[skill.uuid] = skill
        self.updated_attribute('skills','add',skill.uuid)

    def delete_skill(self, uuid):
        if uuid in self._skills.keys():
            self._skills[uuid].remove_from_cache()
            del self._skills[uuid]
            self.updated_attribute('skills','delete',uuid)

    def get_skill(self, uuid):
        return self._skills[uuid]

    def set(self, dct):

        if 'environment' in dct.keys():
            self.environment = NodeParser(dct['environment'], enforce_type=Environment.type_string(trailing_delim=False))

        if 'skills' in dct.keys():
            self.skills = [NodeParser(s, enforce_types=[Skill.type_string(trailing_delim=False)]) for s in dct['skills']]

        super(Program,self).set(dct)

    '''
    Cache Methods
    '''

    def remove_from_cache(self):
        self.environment.remove_from_cache()

        for s in self.skills:
            s.remove_from_cache()

        super(Program,self).remove_from_cache()

    def add_to_cache(self):
        self.environment.add_to_cache()

        for s in self.skills:
            s.add_to_cache()

        super(Program,self).add_to_cache()

    '''
    Children Methods
    '''

    def delete_child(self, uuid):
        success  = False

        if uuid in self.skills_dct.keys():
            self.delete_skill(uuid)
            success = True
        else:
            success = super(Program,self).delete_child(uuid)

        if not success:
            success = self.environment.delete_child(uuid)

        return success

    def add_child(self, node):
        success = False

        if isinstance(node,Skill) and not node.uuid in self.skills_dct.keys():
            self.add_skill(node)
            success = True
        else:
            success = super(Program,self).add_child(node)

        if not success:
            success = self.environment.add_child(node)
        
        return success

    '''
    Update Methods
    '''

    def late_construct_update(self):

        self.environment.late_construct_update()

        for s in self.skills:
            s.late_construct_update()

        super(Program,self).late_construct_update()

        if not self._orphan_list.empty():
            evd_orphan_repair()

    def deep_update(self):

        self.environment.deep_update()

        for s in self.skills:
            s.deep_update()

        super(Program,self).deep_update()

        self.updated_attribute('context','update')
        self.updated_attribute('environment','update')
        self.updated_attribute('skills','update')

    def shallow_update(self):
        super(Program,self).shallow_update()

        self.updated_attribute('context','update')
        self.updated_attribute('environment','update')
        self.updated_attribute('skills','update')

    '''
    Utility methods
    '''

    def child_changed_event(self, attribute_trace):
        if not self._orphan_list.empty():
            evd_orphan_repair()

        if self.changes_cb != None:
            attribute_trace.append(self._child_changed_event_msg(None, 'callback'))
            self.changes_cb(attribute_trace)

    def updated_attribute(self, attribute, verb, child_uuid = None):
        event = [self._child_changed_event_msg(attribute, verb, child_uuid)]

        if not self._orphan_list.empty():
            evd_orphan_repair()

        if self.changes_cb != None:
            self.changes_cb(event)
