import uuid

from abc import ABCMeta, abstractmethod


class Node(object):
    __metaclass__ = ABCMeta

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        self._parent = None
        self._type = None
        self._name = None

        if uuid is None:
            self._uuid = self._generate_uuid(self.type)
        else:
            self._uuid = uuid

        self.parent = parent
        self.type = 'node.'+type if append_type else type
        self.name = name

    @property
    def context(self):
        if self._parent != None:
            return self._parent.context
        else:
            return None

    @property
    def cache(self):
        if self._parent != None:
            return self._parent.cache
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
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('parent','set')])

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

    @abstractmethod
    def to_dct(self):
        return {
            'type': self.type,
            'name': self.name,
            'uuid': self.uuid
        }

    @classmethod
    def from_dct(cls):
        return cls(type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def child_changed_event(self, attribute_trace):
        if self._parent != None:
            attribute_trace.append(self._child_changed_event_msg(None,'callback'))
            self._parent.child_changed_event(attribute_trace)

    def remove_from_cache(self):
        if self._parent != None and self._parent.cache != None:
            self._parent.cache.remove(self.uuid)

    def add_to_cache(self, uuid, node):
        if self._parent != None and self._parent.cache != None:
            self._parent.cache.add(uuid,node)
