from abstract import Node


class Trajectory(Node):

    def __init__(self, startLoc_uuid=None, endLoc_uuid=None,
                 waypoint_uuids=[], trace_uuid=None, name='', uuid=None, active=False,
                 velocity=0, acceleration=0):
        Abstract.__init__(self,'trajectory',name,uuid)
        self.start_location_uuid = startLoc_uuid
        self.end_location_uuid = endLoc_uuid
        self.waypoint_uuids = waypoint_uuids
        self.trace_uuid = trace_uuid
        self.active = active
        self.velocity = velocity
        self.acceleration = acceleration

    def to_dct(self):
        return {
            'start_location_uuid': self.start_location_uuid,
            'end_location_uuid': self.end_location_uuid,
            'type': self.type,
            'uuid': self.uuid,
            'name': self.name,
            'waypoint_uuids': self.waypoint_uuids,
            'trace_uuid': self.trace_uuid,
            'active': self.active,
            'velocity': self.velocity,
            'acceleration': self.acceleration
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            startLoc_uuid=dct['start_location_uuid'],
            endLoc_uuid=dct['end_location_uuid'],
            waypoint_uuids=dct['waypoint_uuids'],
            trace_uuid=dct['trace_uuid'],
            name=dct['name'],
            uuid=dct['uuid'],
            active=dct['active'],
            velocity=dct['velocity'],
            acceleration=dct['acceleration'])
