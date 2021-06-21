'''
Pinch point is an area on the robot that through actuation of a particular joint
will result in unsafe operation. I.e. a human could get pinched within the robot.

This does not handle robot & collision-mesh pinches which are really just a generalization
on collision itself.
'''

from .environment_node import EnvironmentNode
from ..visualizable import VisualizeMarker, ColorTable
from ..data_nodes.geometry import Orientation, Position
from ..node_parser import NodeParser
from ..type_defs import NUMBER_TYPE, ENUM_TYPE, STRING_TYPE


from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class PinchPoint(EnvironmentNode, VisualizeMarker):

    '''
    Constants
    '''

    GOOD_STATE = "good"
    WARN_STATE = "warn"
    ERROR_STATE = "error"

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Pinch Point'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'pinch-point' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return EnvironmentNode.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = EnvironmentNode.template()
        template['fields'].append({
            'type': ENUM_TYPE,
            'key': 'axis',
            'is_uuid': False,
            'is_list': False,
            'enum_values': [
                'x',
                'y',
                'z'
            ]
        })
        template['fields'].append({
            'type': Position.full_type_string(),
            'key': 'offset',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': STRING_TYPE,
            'key': 'link',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'radius',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'length',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, axis='x', offset=None, link='', radius=0.05, length=0.2,
                 type='', name='', parent=None, uuid=None, append_type=True,
                 editable=True, deleteable=True, description=''):
        self._axis = None
        self._offset = None
        self._link = None
        self._radius = None
        self._length = None

        super(PinchPoint,self).__init__(
            type=PinchPoint.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.axis = axis
        self.offset = offset if offset != None else Position(0,0,0, editable=editable, deleteable=False)
        self.link = link
        self.radius = radius
        self.length = length

    def to_dct(self):
        msg = super(PinchPoint,self).to_dct()
        msg.update({
            'axis': self.axis,
            'offset': self.offset.to_dct(),
            'link': self.link,
            'radius': self.radius,
            'length': self.length
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(axis=dct['axis'],
                   offset=NodeParser(dct['offset'], enforce_types=[Position.type_string(trailing_delim=False)]),
                   link=dct['link'],
                   radius=dct['radius'],
                   length=dct['length'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   editable=dct['editable'],
                   deleteable=dct['deleteable'],
                   description=dct['description'],
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, id=0, state='good'):
        # The frame_id should match the joint associated with this pinchpoint
        # The pose for this marker is at origin for that frame

        if state == self.GOOD_STATE:
            color = ColorTable.GOOD_COLOR
        elif state == self.WARN_STATE:
            color = ColorTable.WARN_COLOR
        elif state == self.ERROR_STATE:
            color = ColorTable.ERROR_COLOR

        marker = Marker()
        marker.header.frame_id = self.link
        marker.type = Marker.CYLINDER
        marker.ns = 'pinch_points'
        marker.id = id
        marker.pose.position = self.offset.to_ros()
        marker.pose.orientation = Orientation.from_axis(self.axis)
        marker.scale = Vector3(2*self.radius,2*self.radius,self.length)
        marker.color = color

        return marker

    '''
    Data accessor/modifier methods
    '''

    @property
    def axis(self):
        return self._axis

    @axis.setter
    def axis(self, value):
        if self._axis != value:
            if value == None:
                raise Exception('Axis cannot be None')

            self._axis = value
            self.updated_attribute('axis','set')

    @property
    def link(self):
        return self._link

    @link.setter
    def link(self, value):
        if self._link != value:
            self._link = value
            self.updated_attribute('link','set')

    @property
    def offset(self):
        return self._offset

    @offset.setter
    def offset(self, value):
        if self._offset != value:
            if value == None:
                raise Exception('Offset cannot be none')

            if self._offset != None:
                self._offset.remove_from_cache()

            self._offset = value
            self._offset.parent = self
            self.updated_attribute('offset','set')

    @property
    def radius(self):
        return self._radius

    @radius.setter
    def radius(self, value):
        if self._radius != value:
            if value < 0:
                raise Exception('Radius must be a positive number or zero')
            self._radius = value
            self.updated_attribute('radius','set')

    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, value):
        if self._length != value:
            if value < 0:
                raise Exception('Length must be a positive number or zero')
            self._length = value
            self.updated_attribute('length','set')

    def set(self, dct):

        if 'axis' in dct.keys():
            self.axis = dct['axis']

        if 'offset' in dct.keys():
            self.offset = NodeParser(dct['offset'], enforce_types=[Position.type_string(trailing_delim=False)])

        if 'link' in dct.keys():
            self.link = dct['link']

        if 'radius' in dct.keys():
            self.radius = dct['radius']

        if 'length' in dct.keys():
            self.length = dct['length']

        super(PinchPoint,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        self.offset.remove_from_cache()

        super(PinchPoint,self).remove_from_cache()

    def add_to_cache(self):
        self.offset.add_to_cache()

        super(PinchPoint,self).add_to_cache()

    '''
    Update Methods
    '''

    def late_construct_update(self):
        self.offset.late_construct_update()

        super(PinchPoint,self).late_construct_update()

    def deep_update(self):

        self.offset.deep_update()

        super(PinchPoint,self).deep_update()

        self.updated_attribute('axis', 'update')
        self.updated_attribute('offset','update')
        self.updated_attribute('link','update')
        self.updated_attribute('radius','update')
        self.updated_attribute('length', 'update')

    def shallow_update(self):
        super(PinchPoint,self).shallow_update()

        self.updated_attribute('axis', 'update')
        self.updated_attribute('offset','update')
        self.updated_attribute('link','update')
        self.updated_attribute('radius','update')
        self.updated_attribute('length', 'update')
