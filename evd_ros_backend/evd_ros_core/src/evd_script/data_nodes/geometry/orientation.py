'''
Provides a conveinent interface between low-level orientation implementations
(like ROS) and the EvD AST.

Convience methods are provided.
'''

import tf
import math
import geometry_msgs.msg as ros_msgs

from ...node import Node
from ... import NUMBER_TYPE


class Orientation(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Orientation'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'orientation' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Node.template()
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'x',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'y',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'z',
            'is_uuid': False,
            'is_list': False
        })
        template['fields'].append({
            'type': NUMBER_TYPE,
            'key': 'w',
            'is_uuid': False,
            'is_list': False
        })
        return template

    @classmethod
    def Identity(cls):
        return cls(0,0,0,1)

    def __init__(self, x, y, z, w, type='', name='', uuid=None, parent=None,
                 append_type=True, editable=True, deleteable=True, description=''):
        self._x = None
        self._y = None
        self._z = None
        self._w = None

        super(Orientation,self).__init__(
            type=Orientation.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def to_dct(self):
        msg = super(Orientation,self).to_dct()
        msg.update({
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'w': self.w
        })
        return msg

    def to_simple_dct(self):
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'w': self.w
        }

    def to_list(self, order='xyzw'):
        if order == 'xyzw':
            return [self.x,self.y,self.z,self.w]
        elif order == 'wxyz':
            return [self.w,self.x,self.y,self.z]

    def to_ros(self):
        return ros_msgs.Quaternion(x=self.x,y=self.y,z=self.z,w=self.w)

    @classmethod
    def from_dct(cls, dct):
        return cls(
            x=dct['x'],
            y=dct['y'],
            z=dct['z'],
            w=dct['w'],
            type=dct['type'] if 'type' in dct.keys() else '',
            append_type=not 'type' in dct.keys(),
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')

    @classmethod
    def from_simple_dct(cls, dct):
        return cls(
            x=dct['x'],
            y=dct['y'],
            z=dct['z'],
            w=dct['w'])

    @classmethod
    def from_ros(cls, obj):
        return cls(
            x=obj.x,
            y=obj.y,
            z=obj.z,
            w=obj.w)

    @classmethod
    def from_list(cls, lst, order='xyzw'):
        if order == 'xyzw':
            return cls(
                x=lst[0],
                y=lst[1],
                z=lst[2],
                w=lst[3])
        elif order == 'wxyz':
            return cls(
                x=lst[1],
                y=lst[2],
                z=lst[3],
                w=lst[0])

    def to_euler(self):
        quaternion = self.to_list('xyzw')
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    @classmethod
    def from_euler(cls, rpy):
        quaternion = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
        return cls.from_list(quaternion,'xyzw')

    @classmethod
    def relative_rotation_list(cls, q1, q2): # in xyzw
        q1_inv = [q1[0], q1[1], q1[2], -q1[3]]
        return  tf.transformations.quaternion_multiply(q2, q1_inv)

    @classmethod
    def relative_rotation(cls, orientation1, orientation2):
        q1 = orientation1.to_list()
        q2 = orientation2.to_list()

        qr = cls.relative_rotation_list(q1, q2)

        return cls.from_list(qr)

    @classmethod
    def difference(cls, orientation1, orientation2):
        q1 = orientation1.to_list()
        q2 = orientation2.to_list()

        qr = cls.relative_rotation_list(q1, q2)

        theta = 2 * math.acos(math.abs(qr[3]))
        return theta

    @classmethod
    def from_axis(cls, axis):
        axis = axis.lower()
        orientation = None

        if axis == 'x' or axis == '+x':
            return cls(0,0,0,1)
        elif axis == '-x':
            return cls(-1,0,0,0)
        elif axis == 'y' or axis == '+y':
            return cls(0,0.7071068,0,0.7071068)
        elif axis == '-y':
            return cls(0,-0.7071068,0,0.7071068)
        elif axis == 'z' or axis == '+z':
            return cls(0,0,0.7071068,0.7071068)
        elif axis == '-z':
            return cls(0,0,-0.7071068,0.7071068)
        else:
            raise Exception('axis is not valid')

    @classmethod
    def Unknown(cls):
        return cls(x='?', y='?', z='?', w='?')

    '''
    Data accessor/modifier methods
    '''

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        if self._x != value:
            self._x = value
            self.updated_attribute('x','set')

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        if self._y != value:
            self._y = value
            self.updated_attribute('y','set')

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        if self._z != value:
            self._z = value
            self.updated_attribute('z','set')

    @property
    def w(self):
        return self._w

    @w.setter
    def w(self, value):
        if self._w != value:
            self._w = value
            self.updated_attribute('w','set')

    def set(self, dct):
        x = dct.get('x',None)
        if x != None:
            self.x = x

        y = dct.get('y',None)
        if y != None:
            self.y = y

        z = dct.get('z',None)
        if z != None:
            self.z = z

        w = dct.get('w',None)
        if w != None:
            self.w = w

        super(Orientation,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(Orientation,self).deep_update()

        self.updated_attribute('x','update')
        self.updated_attribute('y','update')
        self.updated_attribute('z','update')
        self.updated_attribute('w','update')

    def shallow_update(self):
        super(Orientation,self).shallow_update()

        self.updated_attribute('x','update')
        self.updated_attribute('y','update')
        self.updated_attribute('z','update')
        self.updated_attribute('w','update')
