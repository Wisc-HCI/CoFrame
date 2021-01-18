import math

from abc import ABCMeta, abstractmethod

from .geometry import Pose, Position, Orientation
from ..visualizable import VisualizeMarker

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

# Regions are just poses with uncertain positions and orientations
# TODO think about what orientation differences mean for these regions

class AbstractRegion(ABCMeta, Pose, VisualizeMarker):

    '''
    Data structure methods
    '''

    def __init__(self, center_position=None, center_orientation=None, type='', name='', uuid=None, parent=None, append_type=True):

        super(AbstractRegion,self).__init__(
            position=center_position,
            orientation=center_orientation,
            type='abstract-region.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    '''
    Data accessor/modifier methods
    '''

    @abstractmethod
    def check_if_pose_within_uncertainty(self, pose):
        return False # Implement this with an actual check


class CubeRegion(AbstractRegion):

    '''
    Data structure methods
    '''

    def __init__(self, center_position=None, center_orientation=None, uncertainty_x=0.1, uncertainty_y=0.1, uncertainty_z=0.1, type='', name='', uuid=None, parent=None, append_type=True):
        self._uncert_x = None
        self._uncert_y = None
        self._uncert_z = None

        super(CubeRegion,self).__init__(
            center_position=center_position,
            center_orientation=center_orientation,
            type='cube-region.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.uncertainty_x = uncertainty_x
        self.uncertainty_y = uncertainty_y
        self.uncertainty_z = uncertainty_z

    def to_dct(self):
        msg = super(CubeRegion,self).to_dct()

        del msg['position']         # overriding with center_position
        del msg['orientation']      # overriding with center_orientation

        msg.update({
            'center_position': self.position.to_dct(),
            'center_orientation': self.orientation.to_dct(),
            'uncertainty_x': self.uncertainty_x,
            'uncertainty_y': self.uncertainty_y,
            'uncertainty_z': self.uncertainty_z
        })

        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            center_position=Position.from_dct(dct['center_position']),
            center_orientation=Orientation.from_dct(dct['center_orientation']),
            uncertainty_x=dct['uncertainty_x'],
            uncertainty_y=dct['uncertainty_y'],
            uncertainty_z=dct['uncertainty_z'],
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')


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

        if 'center_position' in dct.keys():
            self.position = Position.from_dct(dct['center_position'])

        if 'center_orientation' in dct.keys():
            self.orientation = Orientation.from_dct(dct['center_orientation'])

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

        #TODO check orientation later

        return within

    '''
    Update Methods
    '''

    def deep_update(self):
        super(CubeRegion,self).deep_update()

        self.updated_attribute('center_position', 'update')
        self.updated_attribute('center_orientation', 'update')
        self.updated_attribute('uncertainty_x', 'update')
        self.updated_attribute('uncertainty_y', 'update')
        self.updated_attribute('uncertainty_z', 'update')

    def shallow_update(self):
        super(CubeRegion,self).shallow_update()

        self.updated_attribute('center_position', 'update')
        self.updated_attribute('center_orientation', 'update')
        self.updated_attribute('uncertainty_x', 'update')
        self.updated_attribute('uncertainty_y', 'update')
        self.updated_attribute('uncertainty_z', 'update')



class SphereRegion(AbstractRegion):

    '''
    Data structure methods
    '''

    def __init__(self, center_position=None, center_orientation=None, uncertainty_radius=0.1, type='', name='', uuid=None, parent=None, append_type=True):
        self._uncert_radius = None

        super(SphereRegion,self).__init__(
            center_position=center_position,
            center_orientation=center_orientation,
            type='sphere-region.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.uncertainty_radius = uncertainty_radius

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

        if 'center_position' in dct.keys():
            self.position = Position.from_dct(dct['center_position'])

        if 'center_orientation' in dct.keys():
            self.orientation = Orientation.from_dct(dct['center_orientation'])

        if 'uncertainty_radius' in dct.keys():
            self.uncertainty_radius = dct['uncertainty_radius']

        super(SphereRegion,self).set(dct)

    def check_if_pose_within_uncertainty(self, pose):

        difX = pose.position.x - self.position.x
        difY = pose.position.y - self.position.y
        difZ = pose.position.z - self.position.z
        distance = math.sqrt(math.pow(difX,2) + math.pow(difY,2) + math.pow(difZ,2))

        within = True
        within = within and distance <= self.uncertainty_radius

        #TODO check orientation later

        return within

    '''
    Update Methods
    '''

    def deep_update(self):
        super(SphereRegion,self).deep_update()

        self.updated_attribute('center_position', 'update')
        self.updated_attribute('center_orientation', 'update')
        self.updated_attribute('uncertainty_radius', 'update')

    def shallow_update(self):
        super(SphereRegion,self).shallow_update()

        self.updated_attribute('center_position', 'update')
        self.updated_attribute('center_orientation', 'update')
        self.updated_attribute('uncertainty_radius', 'update')
