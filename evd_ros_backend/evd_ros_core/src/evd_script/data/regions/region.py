'''
Regions are just poses with uncertain orientation (and potentially position in child-types)

NOTE, right now regions are on parallel development to waypoints/locations and these two do
not mingle when discussing robots. Future work should address this.
'''

from ...node_parser import NodeParser
from ..geometry import Pose, Position, Orientation
from ...visualizable import VisualizeMarker

class Region(Pose, VisualizeMarker):

    '''
    Class Constants
    '''

    DEFAULT_ORIENTATION_LIMIT = 1

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'region' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Pose.full_type_string() + cls.type_string()

    def __init__(self, center_position=None, center_orientation=None, free_orientation=True,
                 uncertainty_orientation_limit=1, uncertainty_orientation_alt_target=None,
                 type='', name='', uuid=None, parent=None, append_type=True, editable=True,
                 deleteable=True, description=''):
        self._free_ort = None
        self._uncert_ort_lim = None
        self._uncert_ort_target = None

        super(Region,self).__init__(
            position=center_position,
            orientation=center_orientation,
            type=Region.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

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
            center_position=NodeParser(dct['center_position'], enforce_types=[Position.type_string(trailing_delim=False)]),
            center_orientation=NodeParser(dct['center_orientation'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
            free_orientation=dct['free_orientation'],
            uncertainty_orientation_limit=dct['uncertainty_orientation_limit'],
            uncertainty_orientation_alt_target=NodeParser(dct['uncertainty_orientation_alt_target'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
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
    def center_position(self):
        return self.position

    @center_position.setter
    def center_position(self, value):
        self.position = value
        self.updated_attribute("center_position","set")

    @property
    def center_orientation(self):
        return self.orientation

    @center_orientation.setter
    def center_orientation(self, value):
        self.orientation = value
        self.updated_attribute("center_orientation","set")

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

        if 'center_position' in dct.keys():
            self.center_position = NodeParser(dct["center_position"], enforce_types=[Position.type_string(trailing_delim=False)])

        if 'center_orientation' in dct.keys():
            self.center_orientation = NodeParser(dct["center_orientation"], enforce_types=[Orientation.type_string(trailing_delim=False)])

        if 'free_orientation' in dct.keys():
            self.free_orientation = dct['free_orientation']

        if 'uncertainty_orientation_limit' in dct.keys():
            self.uncertainty_orientation_limit = dct['uncertainty_orientation_limit']

        if 'uncertainty_orientation_alt_target' in dct.keys():
            self.uncertainty_orientation_alt_target = NodeParser(dct['uncertainty_orientation_alt_target'], enforce_types=[Orientation.type_string(trailing_delim=False)]) if dct['uncertainty_orientation_alt_target'] != None else None

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

    def late_construct_update(self):

        if self.uncertainty_orientation_alt_target != None:
            self.uncertainty_orientation_alt_target.late_construct_update()

        super(Region,self).late_construct_update()

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
