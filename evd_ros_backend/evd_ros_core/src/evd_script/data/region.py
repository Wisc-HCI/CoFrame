import math

from .geometry import Pose, Position, Orientation
from ..visualizable import VisualizeMarker

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

# Regions are just poses with uncertain positions and orientations

class Region(Pose, VisualizeMarker):

    '''
    Class Constants
    '''

    DEFAULT_ORIENTATION_LIMIT = 1

    '''
    Data structure methods
    '''

    def __init__(self, center_position=None, center_orientation=None, free_orientation=True,
                 uncertainty_orientation_limit=1, uncertainty_orientation_alt_target=None,
                 type='', name='', uuid=None, parent=None, append_type=True):
        self._free_ort = None
        self._uncert_ort_lim = None
        self._uncert_ort_target = None

        super(Region,self).__init__(
            position=center_position,
            orientation=center_orientation,
            type='abstract-region.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.free_orientation = free_orientation
        self.uncertainty_orientation_limit = uncertainty_orientation_limit if not free_orientation else None
        self.uncertainty_orientation_alt_target = uncertainty_orientation_alt_target if not free_orientation else None

    def to_dct(self):
        msg = super(Region,self).to_dct()

        del msg['position']         # overriding with center_position
        del msg['orientation']      # overriding with center_orientation

        msg.update({
            'center_position': self.position.to_dct(),
            'center_orientation': self.orientation.to_dct(),
            'free_orientation': self.free_orientation,
            'uncertainty_orientation_limit': self.uncertainty_orientation_limit,
            'uncertainty_orientation_alt_target': self.uncertainty_orientation_alt_target.to_dct()
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            center_position=Position.from_dct(dct['center_position']),
            center_orientation=Orientation.from_dct(dct['center_orientation']),
            free_orientation=dct['free_orientation'],
            uncertainty_orientation_limit=dct['uncertainty_orientation_limit'],
            uncertainty_orientation_alt_target=Orientation.from_dct(dct['uncertainty_orientation_alt_target']),
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, frame_id='app', id=0):
        return None # Cannot visualize an un-poisitionally-bounded marker

    '''
    Data accessor/modifier methods
    '''

    @property
    def free_orientation(self):
        return self._free_ort

    @free_orientation.setter
    def free_orientation(self, value):
        # Note will destory previous uncertainty values

        if self._free_ort != value:
            if type(value) != type(True):
                raise Exception("Value must be a boolean")

            self._free_ort = value
            if value:
                self.uncertainty_orientation_limit = None
                self.uncertainty_orientation_alt_target = None
            else:
                self.uncertainty_orientation_limit = self.DEFAULT_ORIENTATION_LIMIT
                self.uncertainty_orientation_alt_target = None

            self.updated_attribute('free_orientation','set')

    @property
    def uncertainty_orientation_limit(self):
        return self._uncert_ort_lim

    @uncertainty_orientation_limit.setter
    def uncertainty_orientation_limit(self, value):
        if self._uncert_ort_lim != value:
            if self._free_ort:
                raise Exception('Orientation is set as free, change free_orientation to False first!')

            if value < 0:
                raise Exception('Value must be positive or zero')

            self._uncert_ort_lim = value
            self.updated_attribute('uncertainty_orientation_limit','set')

    @property
    def uncertainty_orientation_alt_target(self):
        return self._uncert_ort_target

    @uncertainty_orientation_alt_target.setter
    def uncertainty_orientation_alt_target(self, value):
        if self._uncert_ort_target != value:
            if self._free_ort:
                raise Exception('Orientation is set as free, change free_orientation to False first!')

            if self._uncert_ort_target != None:
                self._uncert_ort_target.remove_from_cache()

            self._uncert_ort_target = value
            if self._uncert_ort_target != None:
                self._uncert_ort_target.parent = self

            self.updated_attribute('uncertainty_orientation_alt_target','set')

    def set(self, dct):

        if 'free_orientation' in dct.keys():
            self.free_orientation = dct['free_orientation']

        if 'uncertainty_orientation_limit' in dct.keys():
            self.uncertainty_orientation_limit = dct['uncertainty_orientation_limit']

        if 'uncertainty_orientation_alt_target' in dct.keys():
            self.uncertainty_orientation_alt_target = Orientation.from_dct(dct['uncertainty_orientation_alt_target']) if dct['uncertainty_orientation_alt_target'] != None else None

        super(Region,self).set(dct)

    def check_if_pose_within_uncertainty(self, pose):
        if self.free_orientation:
            return True
        else:
            target = self.uncertainty_orientation_alt_target if self.uncertainty_orientation_alt_target != None else self.orientation

            dif = Orientation.difference(pose.orientation,target)

            return dif <= self.uncertainty_orientation_limit

    '''
    Cache Methods
    '''

    def remove_from_cache(self):

        if self.uncertainty_orientation_alt_target != None:
            self.uncertainty_orientation_alt_target.remove_from_cache()

        super(Region,self).remove_from_cache()

    def add_to_cache(self):

        if self.uncertainty_orientation_alt_target != None:
            self.uncertainty_orientation_alt_target.add_to_cache()

        super(Region,self).add_to_cache()

    '''
    Update Methods
    '''

    def deep_update(self):

        if self.uncertainty_orientation_alt_target != None:
            self.uncertainty_orientation_alt_target.deep_update()

        super(Region,self).deep_update()

        self.updated_attribute('center_position', 'update')
        self.updated_attribute('center_orientation', 'update')
        self.updated_attribute('free_orientation','update')
        self.updated_attribute('uncertainty_orientation_limit','update')
        self.updated_attribute('uncertainty_orientation_alt_target','update')

    def shallow_update(self):
        super(Region,self).shallow_update()

        self.updated_attribute('center_position', 'update')
        self.updated_attribute('center_orientation', 'update')
        self.updated_attribute('free_orientation','update')
        self.updated_attribute('uncertainty_orientation_limit','update')
        self.updated_attribute('uncertainty_orientation_alt_target','update')


class CubeRegion(Region):

    '''
    Data structure methods
    '''

    def __init__(self, center_position=None, center_orientation=None, uncertainty_x=0.1,
                 uncertainty_y=0.1, uncertainty_z=0.1, free_orientation=True,
                 uncertainty_orientation_limit=1, uncertainty_orientation_alt_target=None,
                 type='', name='', uuid=None, parent=None, append_type=True):
        self._uncert_x = None
        self._uncert_y = None
        self._uncert_z = None

        super(CubeRegion,self).__init__(
            center_position=center_position,
            center_orientation=center_orientation,
            free_orientation=free_orientation,
            uncertainty_orientation_limit=uncertainty_orientation_limit,
            uncertainty_orientation_alt_target=uncertainty_orientation_alt_target,
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
        msg.update({
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
            free_orientation=dct['free_orientation'],
            uncertainty_orientation_limit=dct['uncertainty_orientation_limit'],
            uncertainty_orientation_alt_target=Orientation.from_dct(dct['uncertainty_orientation_alt_target']),
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
            2 * self.uncertainty_x,
            2 * self.uncertainty_y,
            2 * self.uncertainty_z)
        marker.color = ColorRGBA(0,0,1,1)

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



class SphereRegion(Region):

    '''
    Data structure methods
    '''

    def __init__(self, center_position=None, center_orientation=None, uncertainty_radius=0.1,
                 free_orientation=True, uncertainty_orientation_limit=1, uncertainty_orientation_alt_target=None,
                 type='', name='', uuid=None, parent=None, append_type=True):
        self._uncert_radius = None

        super(SphereRegion,self).__init__(
            center_position=center_position,
            center_orientation=center_orientation,
            free_orientation=free_orientation,
            uncertainty_orientation_limit=uncertainty_orientation_limit,
            uncertainty_orientation_alt_target=uncertainty_orientation_alt_target,
            type='sphere-region.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

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
            center_position=Position.from_dct(dct['center_position']),
            center_orientation=Orientation.from_dct(dct['center_orientation']),
            uncertainty_radius=dct['uncertainty_radius'],
            free_orientation=dct['free_orientation'],
            uncertainty_orientation_limit=dct['uncertainty_orientation_limit'],
            uncertainty_orientation_alt_target=Orientation.from_dct(dct['uncertainty_orientation_alt_target']),
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
        marker.color = ColorRGBA(0,0,1,1)

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

        if 'center_position' in dct.keys():
            self.position = Position.from_dct(dct['center_position'])

        if 'center_orientation' in dct.keys():
            self.orientation = Orientation.from_dct(dct['center_orientation'])

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
