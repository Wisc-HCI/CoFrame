import uuid

from abc import ABCMeta, abstractmethod


class Node(object):
    __metaclass__ = ABCMeta

    def __init__(self, type='', name='', uuid=None, parent=None, append_type=True):
        self._initialize_private_members()
        
        self.parent = parent
        self.type = 'node.'+type if append_type else type
        self.name = name
        if uuid is None:
            self._uuid = self._generate_uuid(self.type)
        else:
            self._uuid = uuid

    def _initialize_private_members(self):
        self._parent = None
        self._type = None
        self._name = None

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
                    [self._child_changed_event_msg('type')])

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        if self._name != value:
            self._name = name
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('name')])

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        if self._parent != value:
            self._parent = parent
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('parent')])

    @staticmethod
    def _generate_uuid(type):
        return '{}-py-{}'.format(type,uuid.uuid1().hex)

    @staticmethod
    def _child_changed_event_msg(attribute):
        return {
            'type': self.type,
            'uuid': self.uuid,
            'attribute': attribute
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
            attribute_trace.append(self._child_changed_event_msg(None))
            self._parent.child_changed_event(attribute_trace)
