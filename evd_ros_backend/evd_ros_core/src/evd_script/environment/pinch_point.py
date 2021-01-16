from ..node import Node
from ..visualizable import VisualizeMarker
from ..data.geometry import Pose, Orientation

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA


class PinchPoint(Node, VisualizeMarker):

    '''
    Constants
    '''

    GOOD_STATE = "good"
    WARN_STATE = "warn"
    ERROR_STATE = "error"

    '''
    Data structure methods
    '''

    def __init__(self, state = None, link='', type='', orientation=None, name='', parent=None, uuid=None, append_type=True):
        self._state = None
        self._orientation = None
        self._link = None

        super(PinchPoint,self).__init__(
            type='pinch-point.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.state = state if state != None else GOOD_STATE
        self.orientation = orientation if orientation != None else Orientation()
        self.link = link

    def to_dct(self):
        msg = super(PinchPoint,self).to_dct()
        msg.update({
            'state': self.state,
            'orientation': self.orientation,
            'link': self.link
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(state=dct['state'],
                   orientation=Orientation.from_dct(dct['orientation']),
                   link=dct['link'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, id=0):
        # The frame_id should match the joint associated with this pinchpoint
        # The pose for this marker is at origin for that frame

        if self.state == self.GOOD_STATE:
            color = ColorRGBA(0,1,0,1)
        elif self.state == self.WARN_STATE:
            color = ColorRGBA(0.5,0.5,0,1)
        elif self.state == self.ERROR_STATE:
            color = ColorRGBA(1,0,0,1)

        marker = Marker()
        marker.header.frame_id = self.link
        marker.type = Marker.CYLINDER
        marker.ns = 'pinch_points'
        marker.id = id
        marker.pose = Pose(orientation=self.orientation).to_ros()
        marker.scale = Vector3(0.1,0.1,0.1)
        marker.color = color

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
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if self._orientation != value:
            if value == None:
                raise Exception('Orientation cannot be None')
            self._orientation = value
            self.updated_attribute('orientation','set')

    @property
    def link(self):
        return self._link

    @link.setter
    def link(self):
        if self._link != value:
            self._link = value
            self.updated_attribute('link','set')

    def set(self, dct):
        if 'state' in dct.keys():
            self.state = dct['state']

        if 'orientation' in dct.keys():
            self.orientation = Orientation.from_dct(dct['orientation'])

        if 'link' in dct.keys():
            self.link = dct['link']

        super(PinchPoint,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(PinchPoint,self).deep_update()

        self.updated_attribute('state','update')
        self.updated_attribute('orientation','update')
        self.updated_attribute('link','update')

    def shallow_update(self):
        super(PinchPoint,self).shallow_update()

        self.updated_attribute('state','update')
        self.updated_attribute('orientation','update')
        self.updated_attribute('link','update')
