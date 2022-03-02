'''
Node is the root type within EvDScript's AST.

Each node provides the basic serialization/deserialization behavior,
a set of public facing properties, a generic set method, potentially child
methods, cache management, and change callback tracing behavior.
'''

import uuid

from abc import ABC
from .cache import *
from .type_defs import STRING_TYPE, BOOLEAN_TYPE

class Node(ABC):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Node'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'node' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return cls.type_string()

    @classmethod
    def template(cls):
        return {
            'type': cls.full_type_string(),
            'name': cls.display_name(),
            'metadata': [
                {
                    'type': STRING_TYPE,
                    'key': 'type'
                },
                {
                    'type': STRING_TYPE,
                    'key': 'uuid'
                },
                {
                    'type': STRING_TYPE,
                    'key': 'name'
                },
                {
                    'type': BOOLEAN_TYPE,
                    'key': 'editable'
                },
                {
                    'type': BOOLEAN_TYPE,
                    'key': 'deleteable'
                },
                {
                    'type': STRING_TYPE,
                    'key': 'description'
                }
            ],
            'fields': []
        }

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True,
                 editable=True, deleteable=True, description=''):
        self._parent = None
        self._type = None
        self._name = None
        self._description = None

        if uuid is None:
            self._uuid = self._generate_uuid(type)
        else:
            self._uuid = uuid

        self.parent = parent
        self.type = Node.type_string() + type if append_type else type
        self.name = name
        self.description = description

        self._editable = editable
        self._deleteable = deleteable
        self.updated_attribute("editable","set") # called this as these are set directly on the private members
        self.updated_attribute("deleteable","set")

    @classmethod
    def from_dct(cls, dct):
        return cls(type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '',
                   editable=dct['editable'] if 'editable' in dct.keys() else True,
                   deleteable=dct['deleteable'] if 'deleteable' in dct.keys() else True,
                   description=dct['description'] if 'description' in dct.keys() else '')

    def to_dct(self):
        return {
            'type': self.type,
            'name': self.name,
            'uuid': self.uuid,
            'editable': self.editable,
            'deleteable': self.deletable,
            'description': self.description
        }

    def on_delete(self):
        pass # Implement this if your node needs to clean up something on deletion

    '''
    Data accessor/modifier methods
    '''

    @property
    def context(self):
        # Since EvD only has a global context, this is more a short-hand of saying the node is
        # situated in a program.
        if self._parent != None:
            return self._parent.context
        else:
            return None

    @property
    def uuid(self):
        return self._uuid

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, value):
        if self._type != value:
            self._type = value
            self.updated_attribute("type","set")

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        if self._name != value:
            self._name = value
            self.updated_attribute("name","set")

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        if self._parent != value:

            self.remove_from_cache()
            self._parent = value
            self.add_to_cache()

            self.updated_attribute("parent","set")

    @property
    def editable(self):
        return self._editable

    @property
    def deletable(self):
        return self._deleteable

    @property
    def description(self):
        return self._description

    @description.setter
    def description(self, value):
        if self._description != value:
            self._description = value
            self.updated_attribute('description','set')

    def set(self, dct):
        # Note: cannot set uuid or parent with this
        # Note: cannot set editable or deleteable status

        name = dct.get('name',None)
        if name != None:
            self.name = name

        type = dct.get('type',None)
        if type != None:
            self.type = type

        if 'description' in dct.keys():
            self.description = dct['description']

    '''
    Cache methods
        - The cache is very important for quick lookup of program nodes.
        - All objects should make sure to add and remove themselves and their
          children as state changes in EvDScript.
    '''

    def remove_from_cache(self):
        get_evd_cache_obj().remove(self.uuid)

    def add_to_cache(self):
        get_evd_cache_obj().add(self._uuid,self)

    def refresh_cache_entry(self):
        self.remove_from_cache()
        self.add_to_cache()

    '''
    Children methods (optional)
        - If nodes encapsulate other nodes then their implementation should
          expose useful variants of these methods.
    '''

    def delete_child(self, uuid):
        # write this for each sub-node type that has set of deletable children
        return False #no children in root node to delete

    def add_child(self, node):
        # write this for each sub-node type that has set of addable children
        return False #no children in root node can be added

    '''
    Utility methods
        - Subnodes probably will not need to override these
    '''

    def get_exact_type(self):
        type = self.type.split('.')
        exactType = type[len(type) - 2]
        return exactType

    @staticmethod
    def _generate_uuid(type):
        return '{}-py-{}'.format(type,uuid.uuid1().hex)

    def child_changed_event(self, attribute_trace):
        if self._parent != None:
            attribute_trace.append(self._child_changed_event_msg(None,'callback'))
            self._parent.child_changed_event(attribute_trace)

    def _child_changed_event_msg(self, attribute, verb, child_uuid = None):
        return {
            'type': self.type,
            'uuid': self.uuid,
            'attribute': attribute,
            'verb': verb,
            'child_uuid': child_uuid
        }

    def __eq__(self, other):
        try:
            return self.uuid == other.uuid
        except:
            return False

    def __del__(self):
        self.on_delete()

    '''
    Update methods
        - Various methods to trigger callback traces and repair state (if needed)
    '''

    def late_construct_update(self):
        # Implement if your class needs to update something after entire program is constructed
        # Note that late construct must be called after a program is complete. This should be
        # handled if using the standard data server and data client.
        pass

    def updated_attribute(self, attribute, verb, child_uuid = None):
        if self._parent != None:
            self._parent.child_changed_event(
                [self._child_changed_event_msg(attribute, verb, child_uuid)])

    def deep_update(self):
        self.updated_attribute('name', 'update')
        self.updated_attribute('type', 'update')
        self.updated_attribute('uuid', 'update')
        self.updated_attribute('editable', 'update')
        self.updated_attribute('deleteable', 'update')
        self.updated_attribute('description','update')

    def shallow_update(self):
        self.updated_attribute('name', 'update')
        self.updated_attribute('type', 'update')
        self.updated_attribute('uuid', 'update')
        self.updated_attribute('editable', 'update')
        self.updated_attribute('deleteable', 'update')
        self.updated_attribute('description','update')

    '''
    Execution methods:
        - These allow for working through the AST at run-time.
    '''

    def symbolic_execution(self, hooks):
        # Implement the pre-post conditions directly

        # EX) Really simple example would look like
        #   hooks.active_primitive = self
        #   return self.parent # Node itself does nothing

        raise Exception('This node is not defined for execution')

    def realtime_execution(self, hooks):
        # Implement the full real-time simulation
        
        # EX) Really simple example would look like
        #   hooks.active_primitive = self
        #   return self.parent # Node itself does nothing
        
        raise Exception('This node is not defined for execution')