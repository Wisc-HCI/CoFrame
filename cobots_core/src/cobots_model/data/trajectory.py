from ..node import Node
from trace import Trace
from waypoint import Waypoint


class Trajectory(Node):

    def __init__(self, startLocUuid=None, endLocUuid=None, waypoints=[],
                 trace=None, velocity=0, acceleration=0, parent=None, type='',
                 name='', uuid=None, append_type=True):
        super(Trajectory,self).__init__(
            type='trajectory.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._start_location_uuid = None
        self._end_location_uuid = None
        self._waypoints = []
        self._velocity = None
        self._acceleration = None
        self._trace = None

        self.start_location_uuid = startLocUuid
        self.end_location_uuid = endLocUuid
        self.waypoints = waypoints
        self.velocity = velocity
        self.acceleration = acceleration
        self.trace = trace

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
                    [self._child_changed_event_msg('start_location_uuid','set')])

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
                    [self._child_changed_event_msg('end_location_uuidy','set')])

    @property
    def waypoints(self):
        return self._waypoints

    @waypoints.setter
    def waypoints(self, value):
        if self._waypoints != value:
            for w in self._waypoints:
                w.remove_from_cache()

            self._waypoints = value
            for w in self._waypoints:
                w.parent = self
                self.add_to_cache(w.uuid,w)

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('waypoints','set')])

    @property
    def trace(self):
        return self._trace

    @trace.setter
    def trace(self, value):
        if self._trace != value:
            if self._trace != None:
                self._trace.remove_from_cache()

            self._trace = value
            if self._trace != None:
                self._trace.parent = self
                self.add_to_cache(self._trace.uuid,self._trace)

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('trace','set')])

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if self._velocity != value:
            self._velocity = value

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('velocity','set')])

    @property
    def acceleration(self):
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value):
        if self._acceleration != value:
            self._acceleration = value

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('acceleration','set')])

    def add_waypoint(self, wp):
        wp.parent = self
        self._waypoints.append(wp)
        self.add_to_cache(wp.uuid,wp)

        self.trace = None

        if self._parent != None:
            self._parent.child_changed_event(
                [self._child_changed_event_msg('waypoints','add')])

    def get_waypoint(self, uuid):
        for w in self.waypoints:
            if w.uuid == uuid:
                return w
        return None

    def reorder_waypoints(self, uuid, shift):
        idx = None
        for i in range(0,len(self._waypoints)):
            if self._waypoints[i].uuid == uuid:
                idx = i
                break

        if idx != None:
            shiftedIdx = idx + shift
            if shiftedIdx < 0 or shiftedIdx >= len(self._waypoints):
                raise Exception("Index out of bounds")

            copy = self._waypoints.pop(idx)
            self._waypoints.insert(shiftedIdx,copy) #TODO check to make sure not off by one

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('waypoints','reorder')])

    def delete_waypoint(self, uuid):
        delIdx = None
        for i in range(0,len(self._waypoints)):
            if self._waypoints[i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._waypoints.pop(i).remove_from_cache()

            self.trace = None

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('waypoints','delete')])

    def to_dct(self):
        msg = super(Trajectory,self).to_dct()
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

    def remove_from_cache(self):
        for w in self._waypoints:
            w.remove_from_cache

        if self._trace != None:
            self._trace.remove_from_cache()

        super(Trajectory,self).remove_from_cache()

    def set(self, dct):
        pass #TODO write this

    def delete_child(self, uuid):
        pass #TODO write this
