from geometry import Pose, Position, Orientation
from ..visualizable import VisualizeMarker

from visualization_msgs.msg import Marker, ColorTable
from geometry_msgs.msg import Vector3


class Waypoint(Pose, VisualizeMarker):

    '''
    Data structure methods
    '''

    def __init__(self, position=None, orientation=None, joints=None, type='',
                 name='', uuid=None, parent=None, append_type=True):

        self._joints = None

        super(Waypoint,self).__init__(
            position=position,
            orientation=orientation,
            type='waypoint.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.joints = joints

    def to_dct(self):
        msg = super(Waypoint,self).to_dct()
        msg.update({
            'joints': self.joints
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            position=Position.from_dct(dct['position']),
            orientation=Orientation.from_dct(dct['orientation']),
            type=dct['type'],
            append_type=False,
            name=dct['name'],
            uuid=dct['uuid'],
            joints=dct['joints'])

    def to_ros_marker(self, frame_id='app', id=0):
        # The frame_id should be the application frame

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.ARROW
        marker.ns = 'waypoints'
        marker.id = id
        marker.pose = self.to_ros()
        marker.scale = Vector3(0.05,0.01,0.01)
        marker.color = ColorTable.WAYPOINT_COLOR

        return marker

    '''
    Data accessor/modifier methods
    '''

    @property
    def joints(self):
        return self._joints

    @joints.setter
    def joints(self, value):
        if self._joints != value:
            self._joints = value
            self.updated_attribute('joints','set')

    def set(self, dct):
        self.joints = dct.get('joints',None)

        super(Waypoint,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(Waypoint,self).deep_update()

        self.updated_attribute('joints','update')

    def shallow_update(self):
        super(Waypoint,self).shallow_update()

        self.updated_attribute('joints','update')
