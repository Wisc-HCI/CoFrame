'''
A region with position uncertainty defined by a sphere.
'''

from .region import Region
from ...node_parser import NodeParser
from ..geometry import Position, Orientation
from ...visualizable import ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class SphereRegion(Region):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'sphere-region' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Region.full_type_string() + cls.type_string()

    def __init__(self, center_position=None, center_orientation=None, uncertainty_radius=0.1,
                 free_orientation=True, uncertainty_orientation_limit=1, uncertainty_orientation_alt_target=None,
                 type='', name='', uuid=None, parent=None, append_type=True, editable=True, deleteable=True):
        self._uncert_radius = None

        super(SphereRegion,self).__init__(
            center_position=center_position,
            center_orientation=center_orientation,
            free_orientation=free_orientation,
            uncertainty_orientation_limit=uncertainty_orientation_limit,
            uncertainty_orientation_alt_target=uncertainty_orientation_alt_target,
            type=SphereRegion.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable)

        self.uncertainty_radius = uncertainty_radius

    def to_dct(self):
        msg = super(SphereRegion,self).to_dct()
        msg.update({
            'uncertainty_radius': self.uncertainty_radius
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            center_position=NodeParser(dct['center_position'], enforce_types=[Position.type_string(trailing_delim=False)]),
            center_orientation=NodeParser(dct['center_orientation'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
            uncertainty_radius=dct['uncertainty_radius'],
            free_orientation=dct['free_orientation'],
            uncertainty_orientation_limit=dct['uncertainty_orientation_limit'],
            uncertainty_orientation_alt_target=NodeParser(dct['uncertainty_orientation_alt_target'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
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
            2 * self.uncertainty_radius,
            2 * self.uncertainty_radius,
            2 * self.uncertainty_radius)
        marker.color = ColorTable.REGION_COLOR

        return marker

    '''
    Data accessor/modifier methods
    '''

    @property
    def uncertainty_radius(self):
        return self._uncert_radius

    @uncertainty_radius.setter
    def uncertainty_radius(self, value):
        if self._uncert_radius != value:
            if value == None:
                raise Exception('Value cannot be None')

            if value < 0:
                raise Exception('Radius must be a positive number or zero')

            self._uncert_radius = value
            self.updated_attribute('uncertainty_radius','set')

    def set(self, dct):

        if 'uncertainty_radius' in dct.keys():
            self.uncertainty_radius = dct['uncertainty_radius']

        super(SphereRegion,self).set(dct)

    def check_if_pose_within_uncertainty(self, pose):

        distance = Position.euclidean_distance(pose.position, self.position)

        within = True
        within = within and distance <= self.uncertainty_radius

        within = within and super(SphereRegion,self).check_if_pose_within_uncertainty()

        return within

    '''
    Update Methods
    '''

    def deep_update(self):
        super(SphereRegion,self).deep_update()

        self.updated_attribute('uncertainty_radius', 'update')

    def shallow_update(self):
        super(SphereRegion,self).shallow_update()

        self.updated_attribute('uncertainty_radius', 'update')
