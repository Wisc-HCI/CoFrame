from abstract import Abstract


class Task(Abstract):

    def __init__(self, name='', uuid=None, primitive_uuids=[]):
        Abstract.__init__(self,'task',name,uuid)
        self.primitive_uuids = primitive_uuids

    def to_dct(self):
        return {
            'uuid': self._uuid,
            'name': self.name,
            'type': self.type,
            'primitive_uuids': self.primitive_uuids
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            primitive_uuids=dct['primitive_uuids']
        )
