from abc import ABCMeta, abstractmethod

class Abstract(object):
    __metaclass__ = ABCMeta

    def __init__(self, type, label='', uuid=None):
        self._type = type
        self.label = label
        if uuid is None:
            self._uuid = self.generate_uuid(self._type)
        else:
            self._uuid = uuid

    @property
    def type(self):
        return self._type

    @property
    def uuid(self):
        return self._uuid

    @staticmethod
    def generate_uuid(type):
        return '{}'.format(type)

    @abstractmethod
    def to_dct(self):
        return {}

    @abstractmethod
    @classmethod
    def from_dct(cls):
        return cls()
