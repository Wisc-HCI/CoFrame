from ..node import Node
from ..data.geometry import Position
from ..visualizable import VisualizeMarker, ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class ReachSphere(Node, VisualizeMarker):

    '''
    Constants
    '''

    GOOD_STATE = "good"
    WARN_STATE = "warn"
    ERROR_STATE = "error"

    '''
    Data structure methods
    '''

    def __init__(self, state = None, radius=1, offset=None, type='', name='',
                 parent=None, uuid=None, append_type=True):
        self._state = None
        self._radius = None
        self._offset = None

        super(ReachSphere,self).__init__(
            type='reach-sphere.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.state = state if state != None else self.GOOD_STATE
        self.radius = radius
        self.offset = offset if offset != None else Position(0,0,0)

    def to_dct(self):
        msg = super(ReachSphere,self).to_dct()
        msg.update({
            'state': self.state,
            'radius': self.radius,
            'offset': self.offset.to_dct()
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(state=dct['state'],
                   radius=dct['radius'],
                   offset=Position.from_dct(dct['offset']),
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, frame_id='app', id=0):
        # The frame_id should be app

        if self.state == self.GOOD_STATE:
            color = ColorTable.GOOD_COLOR
        elif self.state == self.WARN_STATE:
            color = ColorTable.WARN_COLOR
        elif self.state == self.ERROR_STATE:
            color = ColorTable.ERROR_COLOR

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.SPHERE
        marker.ns = 'reach_sphere'
        marker.id = id
        marker.pose.position = self.offset.to_ros()
        marker.pose.orientation.w = 1
        marker.scale = Vector3(self.radius*2,self.radius*2,self.radius*2)
        marker.color = color

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
    def radius(self):
        return self._radius

    @radius.setter
    def radius(self, value):
        if self._radius != value:
            if value < 0:
                raise Exception('Radius must be a postive number')
            self._radius = value
            self.updated_attribute('radius','set')

    @property
    def offset(self):
        return self._offset

    @offset.setter
    def offset(self, value):
        if self._offset != value:
            if self._offset != None:
                self._offset.remove_from_cache()

            self._offset = value
            self._offset.parent = self

            self.updated_attribute('offset','set')

    def set(self, dct):
        if 'state' in dct.keys():
            self.state = dct['state']

        if 'radius' in dct.keys():
            self.radius = dct['radius']

        if 'offset' in dct.keys():
            self.offset = Position.from_dct(dct['offset'])

        super(ReachSphere,self).set(dct)

    '''
    Cache Methods
    '''
    def remove_from_cache(self):

        self.offset.remove_from_cache()

        super(ReachSphere,self).remove_from_cache()

    def add_to_cache(self):

        self.offset.add_to_cache()

        super(ReachSphere,self).add_to_cache()

    '''
    Update Methods
    '''

    def deep_update(self):

        self.offset.deep_update()

        super(ReachSphere,self).deep_update()

        self.updated_attribute('state','update')
        self.updated_attribute('radius','update')
        self.updated_attribute('offset','update')

    def shallow_update(self):
        super(ReachSphere,self).shallow_update()

        self.updated_attribute('state','update')
        self.updated_attribute('radius','update')
        self.updated_attribute('offset','update')
