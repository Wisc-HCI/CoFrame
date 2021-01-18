from waypoint import Waypoint

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA


class Location(Waypoint):

    '''
    Class Constants
    '''

    MESH_LOCATION = 'package://evd_ros_core/markers/SimpleGripperPhycon.stl'

    '''
    Data structure methods
    '''

    def __init__(self, position=None, orientation=None, joints=None, type='',
                 name='', uuid=None, parent=None, append_type=True):
        super(Location,self).__init__(
            position=position,
            orientation=orientation,
            joints=joints,
            type='location.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

    def to_ros_marker(self, frame_id='app', id=0):
        # The frame_id should be the application frame

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.MESH_RESOURCE
        marker.ns = 'locations'
        marker.id = id
        marker.pose = self.to_ros()
        marker.scale = Vector3(0.1,0.1,0.1)
        marker.color = ColorRGBA(173/255.0,216/255.0,230/255.0,1)
        marker.mesh_resource = MESH_LOCATION

        return marker
