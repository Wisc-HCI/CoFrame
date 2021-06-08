'''
Occupancy zone defines a region that an agent will potentially occupy during the
task. A zone can be either for the robot (represented as a reach sphere shadow cast
onto the environment) or a human (as a rectangular area).

Parameterization only makes sense for human agent zones.
'''

from .environment_node import EnvironmentNode
from ..visualizable import VisualizeMarker, ColorTable
from ..data_nodes.geometry import Position
from .. import NUMBER_TYPE, ENUM_TYPE


from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class OccupancyZone(EnvironmentNode, VisualizeMarker):

    '''
    Constants
    '''

    HUMAN_TYPE = "human"
    ROBOT_TYPE = "robot"

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Occupancy Zone'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'occupancy-zone' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return EnvironmentNode.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = EnvironmentNode.template()
        template['fields'].append({
            'type': ENUM_TYPE,
            'key': 'occupancy_type',
            'is_uuid': False,
            'is_list': False,
            'enum_values': [
                cls.HUMAN_TYPE,
                cls.ROBOT_TYPE
            ]
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'position_x',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'position_z',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'scale_x',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'scale_z',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'height',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, occupancyType, posX = 0, posZ = 0, sclX = 1, sclZ = 1,
                 height = 0, type='', name='', parent=None, uuid=None,
                 append_type=True, editable=True, deleteable=True, description=''):
        self._occupancy_type = None
        self._position_x = None
        self._position_z = None
        self._scale_x = None
        self._scale_z = None
        self._height = None

        super(OccupancyZone,self).__init__(
            type=OccupancyZone.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.occupancy_type = occupancyType
        self.position_x = posX
        self.position_z = posZ
        self.scale_x = sclX
        self.scale_z = sclZ
        self.height = height

    def to_dct(self):
        msg = super(OccupancyZone,self).to_dct()
        msg.update({
            'occupancy_type': self.occupancy_type,
            'position_x': self.position_x,
            'position_z': self.position_z,
            'scale_x': self.scale_x,
            'scale_z': self.scale_z,
            'height': self.height
        })
        return msg

    def to_position(self):
        return Position(self.position_x,self.height,self.position_z)

    @classmethod
    def from_dct(cls, dct):
        return cls(occupancyType=dct['occupancy_type'],
                   posX=dct['position_x'],
                   posZ=dct['position_z'],
                   sclX=dct['scale_x'],
                   sclZ=dct['scale_z'],
                   height=dct['height'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   editable=dct['editable'],
                   deleteable=dct['deleteable'],
                   description=dct['description'],
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, frame_id='app', id=0):
        # The frame_id should be the application frame

        if self.occupancy_type == OccupancyZone.HUMAN_TYPE:
            color = ColorTable.OCCUPANCY_ZONE_HUMAN_COLOR
        else:
            color = ColorTable.OCCUPANCY_ZONE_ROBOT_COLOR

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.CUBE
        marker.ns = 'occupancy_zones'
        marker.id = id

        marker.pose.position.x = self.position_x
        marker.pose.position.y = self.position_z
        marker.pose.position.z = self.height

        marker.pose.orientation.w = 1

        marker.scale = Vector3(self.scale_x,self.scale_z,0.001)
        marker.color = color

        return marker

    '''
    Data accessor/modifier methods
    '''

    @property
    def occupancy_type(self):
        return self._occupancy_type

    @occupancy_type.setter
    def occupancy_type(self, value):
        if self._occupancy_type != value:

            if value != self.ROBOT_TYPE and value != self.HUMAN_TYPE:
                raise Exception('Invalid occupancy zone type specified')

            self._occupancy_type = value
            self.updated_attribute('occupancy_type', 'set')

    @property
    def position_x(self):
        return self._position_x

    @position_x.setter
    def position_x(self, value):
        if self._position_x != value:
            self._position_x = value
            self.updated_attribute('position_x', 'set')

    @property
    def position_z(self):
        return self._position_z

    @position_z.setter
    def position_z(self, value):
        if self._position_z != value:
            self._position_z = value
            self.updated_attribute('position_z', 'set')

    @property
    def scale_x(self):
        return self._scale_x

    @scale_x.setter
    def scale_x(self, value):
        if self._scale_x != value:
            self._scale_x = value
            self.updated_attribute('scale_x', 'set')

    @property
    def scale_z(self):
        return self._scale_z

    @scale_z.setter
    def scale_z(self, value):
        if self._scale_z != value:
            self._scale_z = value
            self.updated_attribute('scale_z', 'set')

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, value):
        if self._height != value:
            self._height = value
            self.updated_attribute('height', 'set')

    def set(self, dct):

        if 'occupancy_type' in dct.keys():
            self.occupancy_type = dct['occupancy_type']

        if 'position_x' in dct.keys():
            self.position_x = dct['position_x']

        if 'position_z' in dct.keys():
            self.position_z = dct['position_z']

        if 'scale_x' in dct.keys():
            self.scale_x = dct['scale_x']

        if 'scale_z' in dct.keys():
            self.scale_z = dct['scale_z']

        if 'height' in dct.keys():
            self.height = dct['height']

        super(OccupancyZone,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(OccupancyZone,self).deep_update()

        self.updated_attribute('occupancy_type','update')
        self.updated_attribute('position_x','update')
        self.updated_attribute('position_z','update')
        self.updated_attribute('scale_x','update')
        self.updated_attribute('scale_z','update')
        self.updated_attribute('height','update')

    def shallow_update(self):
        super(OccupancyZone,self).shallow_update()

        self.updated_attribute('occupancy_type','update')
        self.updated_attribute('position_x','update')
        self.updated_attribute('position_z','update')
        self.updated_attribute('scale_x','update')
        self.updated_attribute('scale_z','update')
        self.updated_attribute('height','update')