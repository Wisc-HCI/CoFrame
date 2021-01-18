from ..node import Node
from ..data.geometry import Pose
from ..visualizable import VisualizeMarker

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA


class CollisionMesh(Node,VisualizeMarker):

    '''
    Constants
    '''

    GOOD_STATE = "good"
    WARN_STATE = "warn"
    ERROR_STATE = "error"

    '''
    Data structure methods
    '''

    def __init__(self, state = None, mesh_id = None, pose = None, type='', name='',
                 parent=None, uuid=None, append_type=True):
        self._state = None
        self._mesh_id = None
        self._pose = None

        super(CollisionMesh,self).__init__(
            type='collision-mesh.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.state = state if state != None else GOOD_STATE
        self.mesh_id = mesh_id
        self.pose = pose if pose != None else Pose()

    def to_dct(self):
        msg = super(PinchPoint,self).to_dct()
        msg.update({
            'state': self.state,
            'mesh_id': self.mesh_id,
            'pose': self.pose.to_dct()
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(state=dct['state'],
                   mesh_id=dct['mesh_id'],
                   pose=Pose.from_dct(dct['pose']),
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, frame_id='app', id=0):
        # The frame_id should be the application frame

        if self.state == self.GOOD_STATE:
            color = ColorRGBA(0,1,0,1)
        elif self.state == self.WARN_STATE:
            color = ColorRGBA(0.5,0.5,0,1)
        elif self.state == self.ERROR_STATE:
            color = ColorRGBA(1,0,0,1)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.MESH_RESOURCE if self.mesh_id != None else Marker.CUBE
        marker.ns = 'collision_meshes'
        marker.id = id
        marker.pose = self.pose.to_ros()
        marker.scale = Vector3(1,1,1) if self.mesh_id != None else Vector3(0.1,0.1,0.1)
        marker.color = color

        if self.mesh_id != None:
            marker.mesh_resource = self.mesh_id

        return marker


    '''
    Data accessor/modifier methods
    '''

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        if self._state != value:
            if value != self.GOOD_STATE and value != self.WARN_STATE and value != self.ERROR_STATE:
                raise Exception('Invalid state provided')

            self._state = value
            self.updated_attribute('state','set')

    def set_state_good(self):
        self.state = self.GOOD_STATE

    def set_state_warn(self):
        self.state = self.WARN_STATE

    def set_state_error(self):
        self.state = self.ERROR_STATE

    @property
    def mesh_id(self):
        return self._mesh_id

    @mesh_id.setter
    def mesh_id(self, value):
        if self._mesh_id != value:
            self._mesh_id = value
            self.updated_attribute('mesh_id','set')

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, value):
        if self._pose != value:
            if value == None:
                raise Exception('Pose cannot be None')

            if self._pose != None:
                self._psoe.remove_from_cache()

            self._pose = value
            if self._pose != None:
                self._pose.parent = self

            self.updated_attribute('pose','set')

    def set(self, dct):
        if 'state' in dct.keys():
            self.state = dct['state']

        if 'mesh_id' in dct.keys():
            self.mesh_id = dct['mesh_id']

        if 'pose' in dct.keys():
            self.pose = Pose.from_dct(dct['pose'])

        super(CollisionMesh,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        self.pose.remove_from_cache()

        super(CollisionMesh,self).remove_from_cache()

    def add_to_cache(self):
        self.pose.add_to_cache()

        super(CollisionMesh,self).add_to_cache()

    '''
    Update Methods
    '''

    def deep_update(self):
        super(CollisionMesh,self).deep_update()

        self.updated_attribute('state','update')
        self.updated_attribute('mesh_id','update')
        self.updated_attribute('pose','update')

    def shallow_update(self):
        super(CollisionMesh,self).shallow_update()

        self.updated_attribute('state','update')
        self.updated_attribute('mesh_id','update')
        self.updated_attribute('pose','update')
