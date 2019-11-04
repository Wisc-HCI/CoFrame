from abstract import Abstract
from location import Location
from waypoint import Waypoint
from geometry import Pose

class Trajectory(Abstract):

    def __init__(self, startLoc_uuid=None, stopLoc_uuid=None, waypoint_uuids=[], label='', uuid=None):
        Abstract.__init__(self,'trajectory',label,uuid)
        self.start_location_uuid = startLoc_uuid
        self.stop_location_uuid = stopLoc_uuid
        self.waypoint_uuids = waypoint_uuids

    def to_dct(self):
        return {
            'start_location_uuid': self.start_location_uuid,
            'stop_location_uuid': self.stop_location_uuid,
            'type': self._type,
            'uuid': self._uuid,
            'label': self.label,
            'waypoint_uuids': self.waypoint_uuids
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            startLoc_uuid=dct['start_location_uuid'],
            stopLoc_uuid=dct['stop_location_uuid'],
            waypoint_uuids=['waypoint_uuids'],
            label=dct['label'],
            uuid=dct['uuid'])
