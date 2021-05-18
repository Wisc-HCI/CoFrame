from ..node_parser import NodeParser
from .geometry import Pose, Position, Orientation
from ..visualizable import VisualizeMarker, ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3

# Things are objects being processed in the environment. They can be generated and consumed by machines.
# Useful as a token for verification.

class Thing(Pose):

    '''
    Class Constants
    '''

    DANGEROUS = 0
    SAFE = 1

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'thing' + '.' if trailing_delim else ''

    @classmethod
    def full_type_string(cls):
        return Pose.full_type_string() + cls.type_string()

    def __init__(self, thing_type, safety_level=0, mesh_id=None, position=None, orientation=None,
                 weight=0, type='', name='', parent=None, uuid=None, append_type=True):
        self._thing_type = None
        self._safety_level = None
        self._mesh_id = None
        self._weight = None

        super(Thing,self).__init__(
            position=position,
            orientation=orientation,
            type=Thing.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.thing_type = thing_type
        self.safety_level = safety_level
        self.mesh_id = mesh_id
        self.weight = weight

    def to_dct(self):
        msg = super(Thing,self).to_dct()
        msg.update({
            'thing_type': self.thing_type,
            'safety_level': self.safety_level,
            'mesh_id': self.mesh_id,
            'weight': self.weight
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(thing_type=dct['thing_type'],
                   mesh_id=dct['mesh_id'],
                   safety_level=dct['safety_level'],
                   weight=dct['weight'],
                   position=NodeParser(dct['position'], enforce_type=Position.type_string(trailing_delim=False)),
                   orientation=NodeParser(dct['orientation'], enforce_type=Orientation.type_string(trailing_delim=False)),
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, frame_id='app', id=0):
        # The frame_id should be the application frame
        marker = None

        if self.mesh_id != None:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.type = Marker.MESH_RESOURCE
            marker.ns = 'things'
            marker.id = id
            marker.pose = self.to_ros()
            marker.scale = Vector3(1,1,1)
            marker.color = ColorTable.THING_COLOR
            marker.mesh_resource = self.mesh_id

        return marker

    '''
    Data accessor/modifier methods
    '''

    @property
    def thing_type(self):
        return self._thing_type

    @thing_type.setter
    def thing_type(self, value):
        if self._thing_type != value:
            if value == None:
                raise Exception('Thing type must exist')

            self._thing_type = value
            self.updated_attribute('thing_type','set')

    @property
    def safety_level(self):
        return self._safety_level

    @safety_level.setter
    def safety_level(self, value):
        if self._safety_level != value:
            if value > self.SAFE or value < self.DANGEROUS:
                raise Exception('Safety level must be within range ({0},{1})'.format(self.DANGEROUS,self.SAFE))

            self._safety_level = value
            self.updated_attribute('safety_level','set')

    @property
    def mesh_id(self):
        return self._mesh_id

    @mesh_id.setter
    def mesh_id(self, value):
        if self._mesh_id != value:
            self._mesh_id = value
            self.updated_attribute('mesh_id','set')

    @property
    def weight(self):
        return self._weight

    @weight.setter
    def weight(self, value):
        if self._weight != value:
            self._weight = value
            self.updated_attribute('weight','set')

    def set(self, dct):

        if 'thing_type' in dct.keys():
            self.thing_type = dct['thing_type']

        if 'safety_level' in dct.keys():
            self.safety_level = dct['safety_level']

        if 'mesh_id' in dct.keys():
            self.mesh_id = dct['mesh_id']

        if 'weight' in dct.keys():
            self.weight = dct['weight']

        super(Thing,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):

        super(Thing,self).deep_update()

        self.updated_attribute('thing_type','update')
        self.updated_attribute('safety_level','update')
        self.updated_attribute('mesh_id','update')
        self.updated_attribute('weight','update')

    def shallow_update(self):
        super(Thing,self).shallow_update()

        self.updated_attribute('thing_type','update')
        self.updated_attribute('safety_level','update')
        self.updated_attribute('mesh_id','update')
        self.updated_attribute('weight','update')
