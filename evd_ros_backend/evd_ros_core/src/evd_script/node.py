import uuid

from abc import ABC
from .cache import *


class Node(ABC):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'node' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return cls.type_string()

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        self._parent = None
        self._type = None
        self._name = None

        if uuid is None:
            self._uuid = self._generate_uuid(type)
        else:
            self._uuid = uuid

        self.parent = parent
        self.type = Node.type_string() + type if append_type else type
        self.name = name

    @classmethod
    def from_dct(cls, dct):
        return cls(type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_dct(self):
        return {
            'type': self.type,
            'name': self.name,
            'uuid': self.uuid
        }

    def on_delete(self):
        pass # Implement this if your node needs to clean up something on deletion

    '''
    Data accessor/modifier methods
    '''

    @property
    def context(self):
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

    def set(self, dct):
        # Note: cannot set uuid

        name = dct.get('name',None)
        if name != None:
            self.name = name

        type = dct.get('type',None)
        if type != None:
            self.type = type

    '''
    Cache methods
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
    '''

    def delete_child(self, uuid):
        # write this for each sub-node type that has set of deletable children
        return False #no children in root node to delete

    def add_child(self, dct):
        # write this for each sub-node type that has set of addable children
        return False #no children in root node can be added

    def insert_child(self, dct, idx):
        # write this for each sub-node type that has addable children and ordered lists
        return False # no children in root node can be inserted

    '''
    Utility methods
    '''

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
    '''

    def late_construct_update(self):
        pass # Implement if your class needs to update something after entire program is constructed

    def updated_attribute(self, attribute, verb, child_uuid = None):
        if self._parent != None:
            self._parent.child_changed_event(
                [self._child_changed_event_msg(attribute, verb, child_uuid)])

    def deep_update(self):
        self.updated_attribute('name', 'update')
        self.updated_attribute('type', 'update')
        self.updated_attribute('uuid', 'update')

    def shallow_update(self):
        self.updated_attribute('name', 'update')
        self.updated_attribute('type', 'update')
        self.updated_attribute('uuid', 'update')

    '''
    Execution methods
    '''

    def symbolic_execution(self, hooks):
        pass # Inplement the pre-post conditions directly

    def realtime_execution(self, hooks):
        pass # Implement the full real-time simulation
