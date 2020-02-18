import uuid

from abc import ABCMeta, abstractmethod


class Node(object):
    __metaclass__ = ABCMeta

    def __init__(self, type, name='', uuid=None):
        self.type = type
        self.name = name
        if uuid is None:
            self.uuid = self.generate_uuid(self.type)
        else:
            self.uuid = uuid

    @staticmethod
    def generate_uuid(type):
        return '{}-py-{}'.format(type,uuid.uuid1().hex)

    @abstractmethod
    def to_dct(self):
        return {
            'type': self.type,
            'name': self.name,
            'uuid': self.uuid
        }

    @classmethod
    def from_dct(cls):
        return cls('abstract')
