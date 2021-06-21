'''
Provides a conveinent interface between low-level pose implementations (like ROS) and
the EvD AST.

By default all poses are considered to be in the app frame which can be explicitly stated
by link='app' or assumed by link=''. Link can also default to None in which case the link
property is not being enforced / used for that pose node.
'''

from ...type_defs import STRING_TYPE
import numpy as np
import tf
import geometry_msgs.msg as ros_msgs

from ...node import Node
from .position import Position
from .orientation import Orientation
from ...node_parser import NodeParser


class Pose(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Pose'

    @classmethod
    def type_string(clss, trailing_delim=True):
        return 'pose' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Node.template()
        template['fields'].append({
            'type': Position.full_type_string(),
            'key': 'position',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': Orientation.full_type_string(),
            'key': 'orientation',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': STRING_TYPE,
            'key': 'link',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, position=None, orientation=None, link=None, type='', name='', 
                 uuid=None, parent=None, append_type=True, editable=True, 
                 deleteable=True, description=''):
        self._link = None
        self._position = None
        self._orientation = None

        super(Pose,self).__init__(
            type=Pose.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        if position is None:
            self.position = Position(0,0,0, editable=editable, deleteable=False)
        else:
            self.position = position

        if orientation is None:
            self.orientation = Orientation(0,0,0,1, editable=editable, deleteable=False)
        else:
            self.orientation = orientation

        self.link = link

    def to_dct(self):
        msg = super(Pose,self).to_dct()
        msg.update({
            'link': self.link,
            'position': self.position.to_dct(),
            'orientation': self.orientation.to_dct()
        })
        return msg

    def to_simple_dct(self):
        return {
            'link': self.link,
            'position': self.position.to_simple_dct(),
            'orientation': self.orientation.to_simple_dct()
        }

    def to_ros(self, stamped=False):
        pose = ros_msgs.Pose(position=self.position.to_ros(),
                    orientation=self.orientation.to_ros())

        if stamped:
            msg = ros_msgs.PoseStamped()
            msg.header.frame_id = self.link if self.link != None else 'app'
            msg.pose = pose
            return msg
        else:
            return pose

    @classmethod
    def from_dct(cls, dct):
        return cls(position=NodeParser(dct['position'], enforce_types=[Position.type_string(trailing_delim=False)]),
                   orientation=NodeParser(dct['orientation'], enforce_types=[Orientation.type_string(trailing_delim=False)]),
                   link=dct['link'],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   editable=dct['editable'],
                   deleteable=dct['deleteable'],
                   description=dct['description'],
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    @classmethod
    def from_simple_dct(cls, dct):
        return cls(position=Position.from_simple_dct(dct['position']),
                   orientation=Orientation.from_simple_dct(dct['orientation']),
                   link=dct['link'] if 'link' in dct.keys() else '')

    @classmethod
    def from_ros(cls, obj, stamped=False):
        if stamped:
            return cls(position=Position.from_ros(obj.pose.position),
                    orientation=Orientation.from_ros(obj.pose.orientation),
                    link=obj.header.frame_id)
        else:
            return cls(position=Position.from_ros(obj.position),
                    orientation=Orientation.from_ros(obj.orientation),
                    link=None)

    @classmethod
    def compute_relative(cls, pose1, pose2):
        T1 = pose1.to_matrix()
        T2 = pose2.to_matrix()

        T21 = np.matmul(cls.matrix_inverse(T2),T1)
        return T21, cls.from_matrix(T21)

    def to_matrix(self):
        p = self.position
        q = self.orientation
        T = tf.transformations.quaternion_matrix([q.x,q.y,q.z,q.w])
        T[:3,3] = np.array([p.x,p.y,p.z])
        return T

    @classmethod
    def matrix_inverse(cls, T_in):
        R_in = T_in[:3,:3]
        t_in = T_in[:3,[-1]]
        R_out = R_in.T
        t_out = -np.matmul(R_out,t_in)
        return np.vstack((np.hstack((R_out,t_out)),np.array([0,0,0,1])))

    @classmethod
    def from_matrix(cls, mat):
        q = Orientation(*tf.transformations.quaternion_from_matrix(mat))
        p = Position(*mat[:3,3])
        return cls(p,q)

    @classmethod
    def Unknown(cls):
        return cls(position=Position.Unknown(),orientation=Orientation.Unknown())

    '''
    Data accessor/modifier methods
    '''

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        if self._position != value:
            if value == None:
                raise Exception('Position cannot be None')

            if self._position != None:
                self._position.remove_from_cache()

            self._position = value
            self._position.parent = self
            self.updated_attribute('position','set')

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if self._orientation != value:
            if value == None:
                raise Exception('Orientation cannot be None')

            if self._orientation != None:
                self._orientation.remove_from_cache()

            self._orientation = value
            self._orientation.parent = self
            self.updated_attribute('orientation','set')

    @property
    def link(self):
        return self._link

    @link.setter
    def link(self, value):
        if self._link != value:
            self._link = value
            self.updated_attribute('link')

    def set(self, dct):
        pos = dct.get('position',None)
        if pos != None:
            self.position = NodeParser(pos, enforce_types=[Position.type_string(trailing_delim=False)])

        ort = dct.get('orientation',None)
        if ort != None:
            self.orientation = NodeParser(ort, enforce_types=[Orientation.type_string(trailing_delim=False)])

        link = dct.get('link',None)
        if link != None:
            self.link = link

        super(Pose,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        self.position.remove_from_cache()
        self.orientation.remove_from_cache()

        super(Pose,self).remove_from_cache()

    def add_to_cache(self):
        self.position.add_to_cache()
        self.orientation.add_to_cache()

        super(Pose,self).add_to_cache()

    '''
    Update Methods
    '''

    def late_construct_update(self):
        self.orientation.late_construct_update()
        self.position.late_construct_update()

        super(Pose,self).late_construct_update()

    def deep_update(self):
        self.orientation.deep_update()
        self.position.deep_update()

        super(Pose,self).deep_update()

        self.updated_attribute('position','update')
        self.updated_attribute('orientation','update')
        self.updated_attribute('link','update')

    def shallow_update(self):
        super(Pose,self).shallow_update()

        self.updated_attribute('position','update')
        self.updated_attribute('orientation','update')
        self.updated_attribute('link','update')
