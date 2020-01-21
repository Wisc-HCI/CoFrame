from abstract import Abstract
from geometry import Position, Orientation


class Waypoint(Abstract):

    def __init__(self, position=Position(), orientation=Orientation(), joints=None, name='', uuid=None):
        Abstract.__init__(self,'waypoint',label,uuid)
        self.position = position
        self.orientation = orientation
        self.joints = joints

    def to_dct(self):
        return {
            'uuid': self._uuid,
            'name': self.name,
            'position': self.position.to_dct(),
            'orientation': self.orientation.to_dct(),
            'joints': self.joints
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            position=Position.from_dct(dct['position']),
            orientation=Orientation.from_dct(dct['orientation']),
            name=dct['name'],
            uuid=dct['uuid'],
            joints=dct['joints'])
