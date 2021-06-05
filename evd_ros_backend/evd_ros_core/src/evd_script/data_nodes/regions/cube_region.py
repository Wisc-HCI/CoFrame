'''
A region with a position uncertainty defined by a cuboid.
'''

from ... import NUMBER_TYPE
from .region import Region
from ...node_parser import NodeParser
from ...visualizable import ColorTable
from ..geometry import Position, Orientation

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class CubeRegion(Region):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'cube-region' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Region.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Region.template()
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'uncertainty_x',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'uncertainty_y',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'uncertainty_z',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, center_position=None, center_orientation=None, uncertainty_x=0.1,
                 uncertainty_y=0.1, uncertainty_z=0.1, free_orientation=True,
                 uncertainty_orientation_limit=1, uncertainty_orientation_alt_target=None,
                 type='', name='', uuid=None, parent=None, append_type=True, editable=True,
                 deleteable=True, description=''):
        self._uncert_x = None
        self._uncert_y = None
        self._uncert_z = None

        super(CubeRegion,self).__init__(
            center_position=center_position,
            center_orientation=center_orientation,
            free_orientation=free_orientation,
            uncertainty_orientation_limit=uncertainty_orientation_limit,
            uncertainty_orientation_alt_target=uncertainty_orientation_alt_target,
            type=CubeRegion.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.uncertainty_x = uncertainty_x
        self.uncertainty_y = uncertainty_y
        self.uncertainty_z = uncertainty_z

    def to_dct(self):
        msg = super(CubeRegion,self).to_dct()
        msg.update({
            'uncertainty_x': self.uncertainty_x,
            'uncertainty_y': self.uncertainty_y,
            'uncertainty_z': self.uncertainty_z
        })

        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            center_position=NodeParser(dct['center_position'], enforce_types=[Position.type_string(trailing_delim=False)]),
            center_orientation=NodeParser(dct['center_orientation'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
            uncertainty_x=dct['uncertainty_x'],
            uncertainty_y=dct['uncertainty_y'],
            uncertainty_z=dct['uncertainty_z'],
            free_orientation=dct['free_orientation'],
            uncertainty_orientation_limit=dct['uncertainty_orientation_limit'],
            uncertainty_orientation_alt_target=NodeParser(dct['uncertainty_orientation_alt_target'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, frame_id='app', id=0):
        # The frame_id should be the application frame

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.CUBE
        marker.ns = 'regions'
        marker.id = id
        marker.pose = self.to_ros()
        marker.scale = Vector3(
            2 * self.uncertainty_x,
            2 * self.uncertainty_y,
            2 * self.uncertainty_z)
        marker.color = ColorTable.REGION_COLOR

        return marker

    '''
    Data accessor/modifier methods
    '''

    @property
    def uncertainty_x(self):
        return self._uncert_x

    @uncertainty_x.setter
    def uncertainty_x(self, value):
        if self._uncert_x != value:
            if value == None:
                raise Exception('Value cannot be None')

            self._uncert_x = value
            self.updated_attribute('uncertainty_x','set')

    @property
    def uncertainty_y(self):
        return self._uncert_y

    @uncertainty_y.setter
    def uncertainty_y(self, value):
        if self._uncert_y != value:
            if value == None:
                raise Exception('Value cannot be None')

            self._uncert_y = value
            self.updated_attribute('uncertainty_y','set')

    @property
    def uncertainty_z(self):
        return self._uncert_z

    @uncertainty_z.setter
    def uncertainty_z(self, value):
        if self._uncert_z != value:
            if value == None:
                raise Exception('Value cannot be None')

            self._uncert_z = value
            self.updated_attribute('uncertainty_z','set')

    def set(self, dct):

        if 'uncertainty_x' in dct.keys():
            self.uncertainty_x = dct['uncertainty_x']

        if 'uncertainty_y' in dct.keys():
            self.uncertainty_y = dct['uncertainty_y']

        if 'uncertainty_z' in dct.keys():
            self.uncertainty_z = dct['uncertainty_z']

        super(CubeRegion,self).set(dct)

    def check_if_pose_within_uncertainty(self, pose):
        within = True

        within = within and pose.position.x >= self.position.x - self.uncertainty_x
        within = within and pose.position.x <= self.position.x + self.uncertainty_x

        within = within and pose.position.y >= self.position.y - self.uncertainty_y
        within = within and pose.position.y <= self.position.y + self.uncertainty_y

        within = within and pose.position.z >= self.position.z - self.uncertainty_z
        within = within and pose.position.z <= self.position.z + self.uncertainty_z

        within = within and super(CubeRegion,self).check_if_pose_within_uncertainty()

        return within

    '''
    Update Methods
    '''

    def deep_update(self):
        super(CubeRegion,self).deep_update()

        self.updated_attribute('uncertainty_x', 'update')
        self.updated_attribute('uncertainty_y', 'update')
        self.updated_attribute('uncertainty_z', 'update')

    def shallow_update(self):
        super(CubeRegion,self).shallow_update()

        self.updated_attribute('uncertainty_x', 'update')
        self.updated_attribute('uncertainty_y', 'update')
        self.updated_attribute('uncertainty_z', 'update')
