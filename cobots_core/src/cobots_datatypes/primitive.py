
from abstract import Abstract

#TODO reconsider how this is structured


class Primitive(Abstract):

    def __init__(self, label='', uuid=None):
        Abstract.__init__(self,'primitive',label,uuid)

    def to_dct(self):
        return {
            'type': self._type,
            'uuid': self._uuid,
            'label': self.label,
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            label=dct['label'],
            uuid=dct['uuid']
        )
