'''
Waypoints are poses that a robot can achieve. As such, once planned they
have an associated set of joints.
'''

from ..node_parser import NodeParser
from .geometry import Pose, Position, Orientation, Joints
from ..visualizable import ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class Waypoint(Pose):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Waypoint'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'waypoint' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Pose.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Pose.template()
        template['fields'].append({
            'type': Joints.full_type_string(),
            'key': 'joints',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, position=None, orientation=None, joints=None, type='',
                 name='', uuid=None, parent=None, append_type=True, editable=True,
                 deleteable=True, description=''):

        self._joints = None

        super(Waypoint,self).__init__(
            position=position,
            orientation=orientation,
            type=Waypoint.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.joints = joints

    def to_dct(self):
        msg = super(Waypoint,self).to_dct()
        msg.update({
            'joints': self.joints.to_dct() if self.joints != None else None
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            position=NodeParser(dct['position'], enforce_types=[Position.type_string(trailing_delim=False)]),
            orientation=NodeParser(dct['orientation'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            name=dct['name'],
            uuid=dct['uuid'],
            joints=NodeParser(dct['joints'], enforce_types=[Joints.type_string(trailing_delim=False)]) if dct['joints'] != None else None)

    def to_ros_marker(self, frame_id='app', id=0):
        # The frame_id should be the application frame

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.ARROW
        marker.ns = 'waypoints'
        marker.id = id
        marker.pose = self.to_ros()
        marker.scale = Vector3(0.075,0.03,0.03)
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
            if self._joints != None:
                self._joints.remove_from_cache()

            self._joints = value
            if self._joints != None:
                self._joints.parent = self
                
            self.updated_attribute('joints','set')

    def set(self, dct):

        if 'joints' in dct.keys():
            self.joints = NodeParser(dct['joints'], enforce_types=[Joints.type_string(trailing_delim=False)]) if dct['joints'] != None else None

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
