
from abstract import Abstract
from primitive import Pose
from trajectory import Trajectory


class Therblig(Abstract):

    def __init__(self, pose=Pose(), connections=[], trajectories=[], label='', uuid=None):
        Abstract.__init__(self,'therblig',label,uuid)
        self._pose = pose
        self._connections = connections
        self._trajectories = trajectories

    def to_dct(self):
        return {
            'type': self._type,
            'uuid': self._uuid,
            'label': self.label,
            'pose': self._pose.to_dct(),
            'connections': [c.to_dct() for c in self._connections],
            'trajectories': [t.to_dct() for t in self._trajectories]
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            pose=Pose.from_dct(dct['pose']),
            connections=[TherbligTerminal.from_dct(c) for c in dct['connections']],
            trajectories=[Trajectory.from_dct(t) for t in dct['trajectories']],
            label=dct['label'],
            uuid=dct['uuid']
        )


class TherbligTerminal(Abstract):

    def __init__(self, location_uuid=None, label='', uuid=None):
        Abstract.__init__(self,'therblig-terminal',uuid)
        self.location_uuid = location_uuid

    def to_dct(self):
        return {
            'type': self._type,
            'uuid': self._uuid,
            'label': self.label,
            'location_uuid': self.location_uuid
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            location_uuid=dct['location_uuid'],
            label=dct['label'],
            uuid=dct['uuid'])
