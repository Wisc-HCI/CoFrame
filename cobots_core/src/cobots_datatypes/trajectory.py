from node import Node
from trace import Trace
from waypoint import Waypoint


class Trajectory(Node):

    def __init__(self, startLoc_uuid=None, endLoc_uuid=None, waypoints=[],
                 trace=None, velocity=0, acceleration=0, parent=None, type='',
                 name='', uuid=None, append_type=True):
        super(Node,self).__init__(
            type='trajectory.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        self.start_location_uuid = startLoc_uuid
        self.end_location_uuid = endLoc_uuid
        self.waypoints = waypoints
        self.velocity = velocity
        self.acceleration = acceleration
        self.trace = trace

    def _initialize_private_members(self):
        self._start_location_uuid = None
        self._end_location_uuid = None
        self._waypoints = None
        self._velocity = None
        self._acceleration = None
        self._trace = None

    @property
    def start_location_uuid(self):
        return self._start_location_uuid

    @start_location_uuid.setter
    def start_location_uuid(self, value):
        if self._start_location_uuid != value:
            self._start_location_uuid = value

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('start_location_uuid')])

    @property
    def end_location_uuid(self):
        return self._end_location_uuid

    @end_location_uuid.setter
    def end_location_uuid(self, value):
        if self._end_location_uuid != value:
            self._end_location_uuid = value

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('end_location_uuidy')])

    @property
    def waypoints(self):
        return self._waypoints

    @waypoints.setter
    def waypoints(self, value):
        if self._waypoints != value:
            self._waypoints = value
            for w in self._waypoints:
                w.parent = self

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('waypoints')])

    @property
    def trace(self):
        return self._trace

    @trace.setter
    def trace(self, value):
        if self._trace != value:
            self._trace = value
            if self._trace != None:
                self._trace.parent = self

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('trace')])

    @proptery
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if self._velocity != value:
            self._velocity = value

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('velocity')])

    @property
    def acceleration(self):
        return self._acceleration

    @acceleration.setter(self):
    def acceleration(self, value):
        if self._acceleration != value:
            self._acceleration = value

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('acceleration')])

    def add_waypoint(self, wp):
        wp.parent = self
        self._waypoints.append(wp)

        self.trace = None

        if self._parent != None:
            self._parent.child_changed_event(
                [self._child_changed_event_msg('waypoints')])

    def get_waypoint(self, uuid):
        for w in self.waypoints:
            if w.uuid == uuid:
                return w
        return None

    def delete_waypoint(self, uuid):
        delIdx = None
        for i in range(0,len(self._waypoints)):
            if self._waypoints[i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._waypoints.pop(i)

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('waypoints')])

    def to_dct(self):
        msg = super(Node,self).to_dct()
        msg.update({
            'start_location_uuid': self.start_location_uuid,
            'end_location_uuid': self.end_location_uuid,
            'waypoints': [w.to_dct() for w in self.waypoints],
            'trace': self.trace.to_dct() if self.trace != None else None,
            'velocity': self.velocity,
            'acceleration': self.acceleration
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            startLoc_uuid=dct['start_location_uuid'],
            endLoc_uuid=dct['end_location_uuid'],
            waypoints=[Waypoint.from_dct(w) for w in dct['waypoints']],
            trace=Trace.from_dct(dct['trace']) if dct['trace'] != None else None,
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            velocity=dct['velocity'],
            acceleration=dct['acceleration'])
