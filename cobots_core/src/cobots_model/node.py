import uuid

from abc import ABCMeta, abstractmethod


class Node(object):
    __metaclass__ = ABCMeta

    '''
    Data structure methods
    '''

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        self._parent = None
        self._type = None
        self._name = None

        if uuid is None:
            self._uuid = self._generate_uuid(type)
        else:
            self._uuid = uuid

        self.parent = parent
        self.type = 'node.'+type if append_type else type
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
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('type','set')])

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        if self._name != value:
            self._name = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('name','set')])

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        if self._parent != value:

            self.remove_from_cache()
            self._parent = value
            self.add_to_cache()

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('parent','set')])

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

    @property
    def cache(self):
        if self._parent != None:
            return self._parent.cache
        else:
            return None

    def remove_from_cache(self):
        if self.cache != None:
            self.cache.remove(self.uuid)

    def add_to_cache(self):
        if self.cache != None:
            self.cache.add(self._uuid,self)

    def refresh_cache_entry(self):
        self.remove_from_cache()
        self.add_to_cache()

    '''
    Children methods (optional)
    '''

    def delete_child(self, uuid):
        # write this for each sub-node type that has children
        pass #no children in root node to delete

    def delete_children(self):
        # write this for each sub-node type that has children
        pass #no children in root node to delete

    def child_changed_event(self, attribute_trace):
        if self._parent != None:
            attribute_trace.append(self._child_changed_event_msg(None,'callback'))
            self._parent.child_changed_event(attribute_trace)

    '''
    Utility methods
    '''

    @staticmethod
    def _generate_uuid(type):
        return '{}-py-{}'.format(type,uuid.uuid1().hex)

    def _child_changed_event_msg(self, attribute, verb):
        return {
            'type': self.type,
            'uuid': self.uuid,
            'attribute': attribute,
            'verb': verb
        }

    def __eq__(self, other):
        try:
            return self.uuid == other.uuid
        except:
            return False
