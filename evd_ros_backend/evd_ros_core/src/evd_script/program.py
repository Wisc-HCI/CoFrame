'''
Program is a top-level hierarchical task that wraps execution of robot behavior and provides
hook to global Environment/Context. Additionally, all change traces end at this
root node in the AST. Users can hook into this with a change callback.
'''

from .program_nodes.hierarchical import Hierarchical
from .node_parser import NodeParser
from .environment import Environment

from .orphans import *


class Program(Hierarchical):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'program' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Hierarchical.full_type_string() + cls.type_string()

    def __init__(self, primitives=[], changes_cb=None, name='', type='', uuid=None, append_type=True, environment=None, editable=True, deleteable=True):
        self._orphan_list = evd_orphan_list()
        self.changes_cb = changes_cb
        self._environment = None

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
            deleteable=deleteable)

        self.environment = environment

    def to_dct(self):
        msg = super(Program,self).to_dct()
        msg.update({
            'environment': self.environment.to_dct()
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'],
            primitives=[NodeParser(p) for p in dct['primitives']],
            environment=NodeParser(dct['environment'], enforce_types=[Environment.type_string(trailing_delim=False)]))

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

    def set(self, dct):

        if 'environment' in dct.keys():
            self.environment = NodeParser(dct['environment'], enforce_type=Environment.type_string(trailing_delim=False))

        super(Program,self).set(dct)

    '''
    Cache Methods
    '''

    def remove_from_cache(self):
        self.environment.remove_from_cache()
        super(Program,self).remove_from_cache()

    def add_to_cache(self):
        self.environment.add_to_cache()
        super(Program,self).add_to_cache()

    '''
    Update Methods
    '''

    def late_construct_update(self):

        self.environment.late_construct_update()

        super(Program,self).late_construct_update()

        if not self._orphan_list.empty():
            evd_orphan_repair()

    def deep_update(self):

        self.environment.deep_update()

        super(Program,self).deep_update()

        self.updated_attribute('context','update')
        self.updated_attribute('environment','update')

    def shallow_update(self):
        super(Program,self).shallow_update()

        self.updated_attribute('context','update')
        self.updated_attribute('environment','update')

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
