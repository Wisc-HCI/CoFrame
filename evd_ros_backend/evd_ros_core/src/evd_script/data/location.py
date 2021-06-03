'''
Semantically meaningful waypoints that a robot can operate on as
start/end points in a trajectory.
'''

from .waypoint import Waypoint

from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from ..visualizable import ColorTable


class Location(Waypoint):

    '''
    Class Constants
    '''

    MESH_LOCATION = 'package://evd_ros_core/markers/LocationMarker.stl'

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'location' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Waypoint.full_type_string() + cls.type_string()

    def __init__(self, position=None, orientation=None, joints=None, type='',
                 name='', uuid=None, parent=None, append_type=True, editable=True, deleteable=True):
        super(Location,self).__init__(
            position=position,
            orientation=orientation,
            joints=joints,
            type=Location.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable)

    def to_ros_marker(self, frame_id='app', id=0):
        # The frame_id should be the application frame

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.MESH_RESOURCE
        marker.ns = 'locations'
        marker.id = id
        marker.pose = self.to_ros()
        marker.scale = Vector3(1,1,1)
        marker.color = ColorTable.LOCATION_COLOR
        marker.mesh_resource = self.MESH_LOCATION

        return marker
