'''
Trajectory is used to expose robot movement plannning in EvD.

A trajectory is composed of a start location, an ordered set of waypoints, and an
end location. When a trajectory is planned it produces a trace. Any change to the
trajectory will result in a new trace needing to be computed.
'''

from ..type_defs import NUMBER_TYPE, ENUM_TYPE
from .location import Location
from .waypoint import Waypoint
from ..node import Node
from .trace import Trace
from ..node_parser import NodeParser
from ..visualizable import VisualizeMarkers, VisualizeMarker, ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class Trajectory(Node, VisualizeMarker, VisualizeMarkers):

    TYPES = ['joint', 'ee_ik']

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Trajectory'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'trajectory' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Node.template()
        template['fields'].append({
            'type': Location.full_type_string(),
            'key': 'start_location_uuid',
            'is_uuid': True,
            'is_list': False
        })
        template['field'].append({
            'type': Location.full_type_string(),
            'key': 'end_location_uuid',
            'is_uuid': True,
            'is_list': False
        })
        template['field'].append({
            'type': Waypoint.full_type_string(),
            'key': 'waypoint_uuids',
            'is_uuid': True,
            'is_list': True
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'velocity',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': ENUM_TYPE,
            'key': 'move_type',
            'is_uuid': False,
            'is_list': False,
            'enum_values': [x for x in cls.TYPES]
        })
        template['fields'].append({
            'type': Trace.full_type_string(),
            'key': 'trace',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, startLocUuid=None, endLocUuid=None, waypointUuids=[],
                 trace=None, move_type="joint", velocity=0, parent=None, type='',
                 name='', uuid=None, append_type=True, editable=True,
                 deleteable=True, description=''):

        self._start_location_uuid = None
        self._end_location_uuid = None
        self._waypoint_uuids = None
        self._velocity = None
        self._trace = None
        self._move_type = None

        super(Trajectory,self).__init__(
            type=Trajectory.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.start_location_uuid = startLocUuid
        self.end_location_uuid = endLocUuid
        self.waypoint_uuids = waypointUuids
        self.velocity = velocity
        self.move_type = move_type
        self.trace = trace

    def to_dct(self):
        msg = super(Trajectory,self).to_dct()
        msg.update({
            'start_location_uuid': self.start_location_uuid,
            'end_location_uuid': self.end_location_uuid,
            'waypoint_uuids': self.waypoint_uuids,
            'trace': self.trace.to_dct() if self.trace != None else None,
            'velocity': self.velocity,
            'move_type': self.move_type
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            startLocUuid=dct['start_location_uuid'],
            endLocUuid=dct['end_location_uuid'],
            waypointUuids=dct['waypoint_uuids'],
            trace=NodeParser(dct['trace'], enforce_types=[Trace.type_string(trailing_delim=False)]) if dct['trace'] != None else None,
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            velocity=dct['velocity'],
            move_type=dct['move_type'])

    def to_ros_markers(self, frame_id=None, id_start=0):
        if frame_id == None:
            frame_id = self.link if self.link != "" and self.link != None else "app"
            
        waypoint_markers = []
        waypoint_uuids = []

        lineMarker = Marker()
        lineMarker.header.frame_id = frame_id
        lineMarker.type = Marker.LINE_STRIP
        lineMarker.ns = 'trajectories'
        lineMarker.id = id_start
        lineMarker.scale = Vector3(0.01,0.01,0.01)
        lineMarker.color = ColorTable.TRAJECTORY_COLOR
        lineMarker.pose.orientation.w = 1 # to remove uninitialized quaternion warning

        count = id_start + 1

        startLoc = self.context.get_location(self.start_location_uuid)
        lineMarker.points.append(startLoc.position.to_ros())

        for wpUuid in self.waypoint_uuids:
            wp = self.context.get_waypoint(wpUuid)
            marker = wp.to_ros_marker(frame_id,count)
            lineMarker.points.append(marker.pose.position)
            waypoint_markers.append(marker)
            waypoint_uuids.append(wp.uuid)
            count += 1

        endLoc = self.context.get_location(self.end_location_uuid)
        lineMarker.points.append(endLoc.position.to_ros())

        return lineMarker, waypoint_markers, waypoint_uuids

    def to_ros_marker(self, frame_id=None, id=0):
        lineMarker, _, _ = self.to_ros_markers(frame_id,id)
        return lineMarker

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
    def waypoint_uuids(self):
        return self._waypoint_uuids

    @waypoint_uuids.setter
    def waypoint_uuids(self, value):
        if self._waypoint_uuids != value:
            if value == None:
                raise Exception('Waypoints must be a list not None')

            self._waypoint_uuids = value
            self.trace = None
            self.updated_attribute('waypoint_uuids','set')

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

    def add_waypoint_uuid(self, uuid):
        self._waypoint_uuids.append(uuid)
        self.trace = None
        self.updated_attribute('waypoint_uuids','add',uuid)

    def insert_waypoint_uuid(self, uuid, idx):
        self._waypoint_uuids.insert(idx,uuid)
        self.trace = None
        self.updated_attribute('waypoint_uuids','reorder',uuid)

    def reorder_waypoint_uuids(self, uuid, shift):
        idx = None
        for i in range(0,len(self._waypoint_uuids)):
            if self._waypoint_uuids[i] == uuid:
                idx = i
                break

        if idx != None:
            shiftedIdx = idx + shift
            if shiftedIdx < 0 or shiftedIdx >= len(self._waypoint_uuids):
                raise Exception("Index out of bounds")

            copy = self._waypoint_uuids.pop(idx)
            self._waypoint_uuids.insert(shiftedIdx,copy) #TODO check to make sure not off by one
            self.updated_attribute('waypoint_uuids','reorder')

    def delete_waypoint_uuid(self, uuid):
        delIdx = None
        for i in range(0,len(self._waypoint_uuids)):
            if self._waypoint_uuids[i] == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._waypoint_uuids.pop(i)
            self.trace = None
            self.updated_attribute('waypoint_uuids','delete',uuid)

    def set(self, dct):

        if 'start_location_uuid' in dct.keys():
            self.start_location_uuid = dct['start_location_uuid']

        if 'end_location_uuid' in dct.keys():
            self.end_location_uuid = dct['end_location_uuid']

        if 'waypoint_uuids' in dct.keys():
            self.waypoint_uuids = dct['waypoint_uuids']

        velocity = dct.get('velocity',None)
        if velocity != None:
            self.velocity = velocity

        move_type = dct.get('move_type',None)
        if move_type != None:
            self.move_type = move_type

        if 'trace' in dct.keys():
            self.trace = NodeParser(dct['trace'], enforce_types=[Trace.type_string(trailing_delim=False)]) if dct['trace'] != None else None

        super(Trajectory,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):

        if self._trace != None:
            self._trace.remove_from_cache()

        super(Trajectory,self).remove_from_cache()

    def add_to_cache(self):

        if self._trace != None:
            self._trace.add_to_cache()

        super(Trajectory,self).add_to_cache()

    '''
    Update Methods
    '''

    def late_construct_update(self):

        if self.trace != None:
            self.trace.late_construct_update()

        super(Trajectory,self).late_construct_update()

    def deep_update(self):

        if self.trace != None:
            self.trace.deep_update()

        super(Trajectory,self).deep_update()

        self.updated_attribute('start_location_uuid','update')
        self.updated_attribute('end_location_uuid','update')
        self.updated_attribute('waypoint_uuids','update')
        self.updated_attribute('velocity','update')
        self.updated_attribute('move_type','update')
        self.updated_attribute('trace','update')

    def shallow_update(self):
        super(Trajectory,self).shallow_update()

        self.updated_attribute('start_location_uuid','update')
        self.updated_attribute('end_location_uuid','update')
        self.updated_attribute('waypoint_uuids','update')
        self.updated_attribute('velocity','update')
        self.updated_attribute('move_type','update')
        self.updated_attribute('trace','update')