
from abstract import Abstract
from pose import Pose


class Location(Abstract):

    def __init__(self, marker_type='', pose=Pose(), label='', uuid=None):
        Abstract.__init__(self,'location',label,uuid)
        self.pose = pose
        self.marker_type = marker_type

    def to_dct(self):
        return {
            'type': self._type,
            'uuid': self._uuid,
            'label': self.label,
            'pose': self.pose.to_dct(),
            'marker_type': self.marker_type
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            marker_type=dct['marker_type'],
            pose=Pose.from_dct(dct['pose']),
            label=dct['label'],
            uuid=dct['uuid'])
