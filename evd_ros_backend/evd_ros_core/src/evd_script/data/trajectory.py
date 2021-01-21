from ..node import Node
from waypoint import Waypoint
from trace import Trace, TraceDataPoint
from ..visualizable import VisualizeMarkers, ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class Trajectory(Node, VisualizeMarkers):

    TYPES = ['joint', 'linear', 'planner']

    '''
    Data structure methods
    '''

    def __init__(self, startLocUuid=None, endLocUuid=None, waypoints=[],
                 trace=None, move_type="joint", velocity=0, acceleration=0,
                 parent=None, type='', name='', uuid=None, append_type=True):

        self._start_location_uuid = None
        self._end_location_uuid = None
        self._waypoints = None
        self._velocity = None
        self._acceleration = None
        self._trace = None
        self._move_type = None

        super(Trajectory,self).__init__(
            type='trajectory.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.start_location_uuid = startLocUuid
        self.end_location_uuid = endLocUuid
        self.waypoints = waypoints
        self.velocity = velocity
        self.acceleration = acceleration
        self.move_type = move_type
        self.trace = trace

    def to_dct(self):
        msg = super(Trajectory,self).to_dct()
        msg.update({
            'start_location_uuid': self.start_location_uuid,
            'end_location_uuid': self.end_location_uuid,
            'waypoints': [w.to_dct() for w in self.waypoints],
            'trace': self.trace.to_dct() if self.trace != None else None,
            'velocity': self.velocity,
            'acceleration': self.acceleration,
            'move_type': self.move_type
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            startLocUuid=dct['start_location_uuid'],
            endLocUuid=dct['end_location_uuid'],
            waypoints=[Waypoint.from_dct(w) for w in dct['waypoints']],
            trace=Trace.from_dct(dct['trace']) if dct['trace'] != None else None,
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            velocity=dct['velocity'],
            acceleration=dct['acceleration'],
            move_type=dct['move_type'])

    def to_ros_markers(self, frame_id, id_start=0):
        waypoint_markers = []
        waypoint_uuids = []

        lineMarker = Marker()
        lineMarker.header.frame_id = frame_id
        lineMarker.type = Marker.LINE_STRIP
        lineMarker.ns = 'trajectories'
        lineMarker.id = id_start
        lineMarker.scale = Vector3(0.01,0.01,0.01)
        lineMarker.color = ColorTable.TRAJECTORY_COLOR

        count = id_start + 1

        startLoc = self.context.get_location(self.start_location_uuid)
        lineMarker.points.append(startLoc.position.to_ros())

        for wp in self.waypoints:
            marker = wp.to_ros_marker(frame_id,count)
            lineMarker.points.append(marker.pose.position)
            waypoint_markers.append(marker)
            waypoint_uuids.append(wp.uuid)
            count += 1

        endLoc = self.context.get_location(self.end_location_uuid)
        lineMarker.points.append(endLoc.position.to_ros())

        return lineMarker, waypoint_markers, waypoint_uuids

    '''
    Data accessor/modifier methods
    '''

    @property
    def start_location_uuid(self):
        return self._start_location_uuid

    @start_location_uuid.setter
    def start_location_uuid(self, value):
        if self._start_location_uuid != value:
            self._start_location_uuid = value
            self.trace = None
            self.updated_attribute('start_location_uuid','set')

    @property
    def end_location_uuid(self):
        return self._end_location_uuid

    @end_location_uuid.setter
    def end_location_uuid(self, value):
        if self._end_location_uuid != value:
            self._end_location_uuid = value
            self.trace = None
            self.updated_attribute('end_location_uuidy','set')

    @property
    def waypoints(self):
        return self._waypoints

    @waypoints.setter
    def waypoints(self, value):
        if self._waypoints != value:
            if value == None:
                raise Exception('Waypoints must be a list not None')

            if self._waypoints != None:
                for w in self._waypoints:
                    w.remove_from_cache()

            self._waypoints = value
            for w in self._waypoints:
                w.parent = self

            self.trace = None
            self.updated_attribute('waypoints','set')

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

            self.updated_attribute('trace','set')

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if self._velocity != value:
            self._velocity = value
            self.trace = None
            self.updated_attribute('velocity','set')

    @property
    def acceleration(self):
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value):
        if self._acceleration != value:
            self._acceleration = value
            self.trace = None
            self.updated_attribute('acceleration','set')

    @property
    def move_type(self):
        return self._move_type

    @move_type.setter
    def move_type(self, value):
        if self._move_type != value:

            if not value in self.TYPES:
                raise Exception("Invalid move_type provided")

            self._move_type = value
            self.trace = None
            self.updated_attribute('move_type','set')

    def add_waypoint(self, wp):
        wp.parent = self
        self._waypoints.append(wp)
        self.trace = None
        self.updated_attribute('waypoints','add')

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
            self.updated_attribute('waypoints','reorder')

    def delete_waypoint(self, uuid):
        delIdx = None
        for i in range(0,len(self._waypoints)):
            if self._waypoints[i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._waypoints.pop(i).remove_from_cache()
            self.trace = None
            self.updated_attribute('waypoints','delete')

    def set(self, dct):

        if 'start_location_uuid' in dct.keys():
            self.start_location_uuid = dct['start_location_uuid']

        if 'end_location_uuid' in dct.keys():
            self.end_location_uuid = dct['end_location_uuid']

        if 'waypoints' in dct.keys():
            self.waypoints = [Waypoint.from_dct(w) for w in dct['waypoints']]

        velocity = dct.get('velocity',None)
        if velocity != None:
            self.velocity = velocity

        acceleration = dct.get('acceleration',None)
        if acceleration != None:
            self.acceleration = acceleration

        move_type = dct.get('move_type',None)
        if move_type != None:
            self.move_type = move_type

        if 'trace' in dct.keys():
            self.trace = Trace.from_dct(dct['trace']) if dct['trace'] != None else None

        super(Trajectory,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for w in self._waypoints:
            w.remove_from_cache()

        if self._trace != None:
            self._trace.remove_from_cache()

        super(Trajectory,self).remove_from_cache()

    def add_to_cache(self):
        for w in self._waypoints:
            w.add_to_cache()

        if self._trace != None:
            self._trace.add_to_cache()

        super(Trajectory,self).add_to_cache()

    '''
    Children methods
    '''

    def delete_child(self, uuid):
        success = False

        if self.trace != None and self.trace.uuid == uuid:
            self.trace = None
            success = True
        elif uuid in [wp.uuid for wp in self.waypoints]:
            self.delete_waypoint(uuid)
            success = True

        return success

    '''
    Update Methods
    '''

    def deep_update(self):
        for w in self.waypoints:
            w.deep_update()

        if self.trace != None:
            self.trace.deep_update()

        super(Trajectory,self).deep_update()

        self.updated_attribute('start_location_uuid','update')
        self.updated_attribute('end_location_uuid','update')
        self.updated_attribute('waypoints','update')
        self.updated_attribute('velocity','update')
        self.updated_attribute('acceleration','update')
        self.updated_attribute('move_type','update')
        self.updated_attribute('trace','update')

    def shallow_update(self):
        super(Trajectory,self).shallow_update()

        self.updated_attribute('start_location_uuid','update')
        self.updated_attribute('end_location_uuid','update')
        self.updated_attribute('waypoints','update')
        self.updated_attribute('velocity','update')
        self.updated_attribute('acceleration','update')
        self.updated_attribute('move_type','update')
        self.updated_attribute('trace','update')
