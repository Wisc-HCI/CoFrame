from ..node import Node
from ..data.geometry import Pose
from ..visualizable import VisualizeMarker, ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


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

    def __init__(self, mesh_id = None, pose_offset = None, link='', type='', name='',
                 parent=None, uuid=None, append_type=True):
        self._mesh_id = None
        self._pose_offset = None
        self._link = None

        super(CollisionMesh,self).__init__(
            type='collision-mesh.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.mesh_id = mesh_id
        self.pose_offset = pose_offset if pose_offset != None else Pose()
        self.link = link

    def to_dct(self):
        msg = super(CollisionMesh,self).to_dct()
        msg.update({
            'mesh_id': self.mesh_id,
            'pose_offset': self.pose_offset.to_dct(),
            'link': self.link
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(mesh_id=dct['mesh_id'],
                   pose_offset=Pose.from_dct(dct['pose_offset']),
                   link=dct['link'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, id=0, state='good'):
        # The frame_id should be the application frame

        if state == self.GOOD_STATE:
            color = ColorTable.GOOD_COLOR
        elif state == self.WARN_STATE:
            color = ColorTable.WARN_COLOR
        elif state == self.ERROR_STATE:
            color = ColorTable.ERROR_COLOR

        marker = Marker()
        marker.header.frame_id = self.link
        marker.type = Marker.MESH_RESOURCE if self.mesh_id != None else Marker.CUBE
        marker.ns = 'collision_meshes'
        marker.id = id
        marker.pose = self.pose_offset.to_ros()
        marker.scale = Vector3(1,1,1) if self.mesh_id != None else Vector3(0.1,0.1,0.1)
        marker.color = color

        if self.mesh_id != None:
            marker.mesh_resource = self.mesh_id

        return marker


    '''
    Data accessor/modifier methods
    '''

    @property
    def mesh_id(self):
        return self._mesh_id

    @mesh_id.setter
    def mesh_id(self, value):
        if self._mesh_id != value:
            self._mesh_id = value
            self.updated_attribute('mesh_id','set')

    @property
    def pose_offset(self):
        return self._pose_offset

    @pose_offset.setter
    def pose_offset(self, value):
        if self._pose_offset != value:
            if value == None:
                raise Exception('pose_offset cannot be None')

            if self._pose_offset != None:
                self._pose_offset.remove_from_cache()

            self._pose_offset = value
            if self._pose_offset != None:
                self._pose_offset.parent = self

            self.updated_attribute('pose_offset','set')

    @property
    def link(self):
        return self._link

    @link.setter
    def link(self, value):
        if self._link != value:
            self._link = value
            self.updated_attribute('link','set')

    def set(self, dct):

        if 'mesh_id' in dct.keys():
            self.mesh_id = dct['mesh_id']

        if 'pose_offset' in dct.keys():
            self.pose_offset = Pose.from_dct(dct['pose_offset'])

        if 'link' in dct.keys():
            self.link = dct['link']

        super(CollisionMesh,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        self.pose_offset.remove_from_cache()

        super(CollisionMesh,self).remove_from_cache()

    def add_to_cache(self):
        self.pose_offset.add_to_cache()

        super(CollisionMesh,self).add_to_cache()

    '''
    Update Methods
    '''

    def deep_update(self):
        self.pose_offset.deep_update()

        super(CollisionMesh,self).deep_update()

        self.updated_attribute('mesh_id','update')
        self.updated_attribute('pose_offset','update')
        self.updated_attribute('link','update')

    def shallow_update(self):
        super(CollisionMesh,self).shallow_update()

        self.updated_attribute('mesh_id','update')
        self.updated_attribute('pose_offset','update')
        self.updated_attribute('link','update')
