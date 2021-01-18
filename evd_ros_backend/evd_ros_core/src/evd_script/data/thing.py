from ..node import Node
from ..visualizable import VisualizeMarker

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

# Things are objects being processed in the environment. They can be generated and consumed by machines.
# Useful as a token for verification.

class Thing(Node):

    '''
    Class Constants
    '''

    DANGEROUS = 0
    SAFE = 1

    '''
    Data structure methods
    '''

    def __init__(self, thing_type, pose, safety_level=0, mesh_id=None, type='',
                 name='', parent=None, uuid=None, append_type=True):
        self._thing_type = None
        self._pose = None
        self._safety_level = None
        self._mesh_id = None

        super(Thing,self).__init__(
            type='thing.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.thing_type = thing_type
        self.pose = pose
        self.safety_level = safety_level
        self.mesh_id = mesh_id

    def to_dct(self):
        msg = super(Thing,self).to_dct()
        msg.update({
            'thing_type': self.thing_type,
            'pose': self.pose.to_dct(),
            'safety_level': self.safety_level,
            'mesh_id': self.mesh_id
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(thing_type=dct['thing_type']
                   mesh_id=dct['mesh_id'],
                   pose=Pose.from_dct(dct['pose']),
                   safety_level=dct['safety_level'],
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
            marker.pose = self.pose.to_ros()
            marker.scale = Vector3(1,1,1)
            marker.color = ColorRGBA(1,1,1,1)
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
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, value):
        if self._pose != value:
            if value == None:
                raise Exception('Must supply a valid pose')

            if self._pose != None:
                self._pose.remove_from_cache()

            self._pose = value
            self._pose.parent = self
            self.updated_attribute('pose','set')

    @property
    def safety_level(self):
        return self._safety_level

    @safety_level.setter
    def safety_level(self, value):
        if self._safety_level != value:
            if value > self.SAFE or value < self.DANGEROUS:
                raise Exception('Safety levle must be within range ({0},{1})'.format(self.DANGEROUS,self.SAFE))

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

    def set(self, dct):

        if 'thing_type' in dct.keys():
            self.thing_type = dct['thing_type']

        if 'pose' in dct.keys():
            self.pose = Pose.from_dct(dct['pose'])

        if 'safety_level' in dct.keys():
            self.safety_level = dct['safety_level']

        if 'mesh_id' in dct.keys():
            self.mesh_id = dct['mesh_id']

        super(Thing,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):

        self.pose.remove_from_cache()

        super(Thing,self).remove_from_cache()

    def add_to_cache(self):

        self.pose.add_to_cache()

        super(Thing,self).add_to_cache()

    '''
    Update Methods
    '''

    def deep_update(self):

        self.pose.deep_update()

        super(Thing,self).deep_update()

        self.updated_attribute('thing_type','update')
        self.updated_attribute('pose','update')
        self.updated_attribute('safety_level','update')
        self.updated_attribute('mesh_id','update')

    def shallow_update(self):
        super(Thing,self).shallow_update()

        self.updated_attribute('thing_type','update')
        self.updated_attribute('pose','update')
        self.updated_attribute('safety_level','update')
        self.updated_attribute('mesh_id','update')
