from ..node import Node
from ..data.geometry import Pose

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA


class ReachSphere(Node):

    '''
    Constants
    '''

    GOOD_STATE = "good"
    WARN_STATE = "warn"
    ERROR_STATE = "error"

    '''
    Data structure methods
    '''

    def __init__(self, state = None, type='', name='', parent=None, uuid=None, append_type=True):
        self._state = None

        super(ReachSphere,self).__init__(
            type='reach-sphere.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.state = state if state != None else GOOD_STATE

    def to_dct(self):
        msg = super(ReachSphere,self).to_dct()
        msg.update({
            'state': self.state
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(state=dct['state'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    def to_ros_marker(self, frame_id='base_link', id=0):
        # The frame_id should be the robot's base_link

        if self.state == self.GOOD_STATE:
            color = ColorRGBA(0,1,0,1)
        elif self.state == self.WARN_STATE:
            color = ColorRGBA(0.5,0.5,0,1)
        elif self.state == self.ERROR_STATE:
            color = ColorRGBA(1,0,0,1)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.SPHERE
        marker.ns = 'reach_sphere'
        marker.id = id
        marker.pose = Pose().to_ros()
        marker.scale = Vector3(1,1,1)
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

    def set(self, dct):
        if 'state' in dct.keys():
            self.state = dct['state']

        super(ReachSphere,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(ReachSphere,self).deep_update()

        self.updated_attribute('state','update')

    def shallow_update(self):
        super(ReachSphere,self).shallow_update()

        self.updated_attribute('state','update')
