from abstract import Abstract
from geometry import Pose


class Waypoint(Abstract):

    def __init__(self, location_uuid=None, parameters={}, pose=Pose(), label='', uuid=None):
        Abstract.__init__(self,'waypoint',label,uuid)
        self.location_uuid = location_uuid
        self.parameters = parameters
        self.pose = pose

    def to_dct(self):
        return {
            'location_uuid': self.location_uuid,
            'parameters': self.parameters,
            'type': self._type,
            'uuid': self._uuid,
            'label': self.label,
            'pose': self.pose.to_dct()
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            location_uuid=dct['location_uuid'],
            parameters=dct['parameters'],
            pose=Pose.from_dct(dct['pose']),
            label=dct['label'],
            uuid=dct['uuid'])
