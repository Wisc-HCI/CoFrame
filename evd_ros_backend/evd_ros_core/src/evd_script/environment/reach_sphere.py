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

    def __init__(self, radius=1, offset=None, type='', name='',
                 parent=None, uuid=None, append_type=True):
        self._radius = None
        self._offset = None

        super(ReachSphere,self).__init__(
            type='reach-sphere.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.radius = radius
        self.offset = offset if offset != None else Position(0,0,0)

    def to_dct(self):
        msg = super(ReachSphere,self).to_dct()
        msg.update({
            'radius': self.radius,
            'offset': self.offset.to_dct()
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(radius=dct['radius'],
                   offset=Position.from_dct(dct['offset']),
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, frame_id='app', id=0, state='good'):
        # The frame_id should be app

        if state == self.GOOD_STATE:
            color = ColorTable.GOOD_COLOR
        elif state == self.WARN_STATE:
            color = ColorTable.WARN_COLOR
        elif state == self.ERROR_STATE:
            color = ColorTable.ERROR_COLOR
        else:
            raise Exception('State {} is not a valid state'.format(state))

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

    def late_construct_update(self):
        self.offset.late_construct_update()

        super(ReachSphere,self).late_construct_update()

    def deep_update(self):

        self.offset.deep_update()

        super(ReachSphere,self).deep_update()

        self.updated_attribute('radius','update')
        self.updated_attribute('offset','update')

    def shallow_update(self):
        super(ReachSphere,self).shallow_update()

        self.updated_attribute('radius','update')
        self.updated_attribute('offset','update')
