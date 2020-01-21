from abstract import Abstract


class Trajectory(Abstract):

    def __init__(self, type=None, startLoc_uuid=None, endLoc_uuid=None,
                 waypoint_uuids=[], name='', uuid=None, active=False,
                 velocity=0, acceleration=0):
        Abstract.__init__(self,type,name,uuid)
        self.start_location_uuid = startLoc_uuid
        self.end_location_uuid = endLoc_uuid
        self.waypoint_uuids = waypoint_uuids
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
            'active': self.active,
            'velocity': self.velocity,
            'acceleration': self.acceleration
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            startLoc_uuid=dct['start_location_uuid'],
            endLoc_uuid=dct['end_location_uuid'],
            waypoint_uuids=['waypoint_uuids'],
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            active=dct['active'],
            velocity=dct['velocity'],
            acceleration=dct['acceleration'])
