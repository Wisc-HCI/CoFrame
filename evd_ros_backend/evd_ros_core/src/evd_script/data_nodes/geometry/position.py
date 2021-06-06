'''
Provides a conveinent interface between low-level position implementations
(like ROS) and the EvD AST.

Convience methods are provided.
'''

import math
import geometry_msgs.msg as ros_msgs

from ...node import Node
from ... import NUMBER_TYPE


class Position(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Position'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'position' + ('.' if trailing_delim else '')

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
        return template

    def __init__(self, x, y, z, type='', name='', parent=None, uuid=None,
                 append_type=True, editable=True, deleteable=True, description=''):
        self._x = None
        self._y = None
        self._z = None

        super(Position,self).__init__(
            type=Position.type_string() + type if append_type else type,
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

    def to_dct(self):
        msg = super(Position,self).to_dct()
        msg.update({
            'x': self.x,
            'y': self.y,
            'z': self.z
        })
        return msg

    def to_simple_dct(self):
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z
        }

    def to_list(self):
        return [self.x,self.y,self.z]

    def to_ros(self):
        return ros_msgs.Vector3(x=self.x,y=self.y,z=self.z)

    @classmethod
    def from_dct(cls, dct):
        return cls(
            x=dct['x'],
            y=dct['y'],
            z=dct['z'],
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
            z=dct['z'])

    @classmethod
    def from_ros(cls, obj):
        return cls(
            x=obj.x,
            y=obj.y,
            z=obj.z)

    @classmethod
    def from_list(cls, lst):
        return cls(
            x=lst[0],
            y=lst[1],
            z=lst[2])

    @classmethod
    def euclidean_distance(cls, pos1, pos2):
        difX = pos1.x - pos2.x
        difY = pos1.y - pos2.y
        difZ = pos1.z - pos2.z

        distance = math.sqrt(math.pow(difX,2) + math.pow(difY,2) + math.pow(difZ,2))
        return distance

    @classmethod
    def from_axis(cls, axis, scale=1):
        axis = axis.lower()
        orientation = None

        if axis == 'x' or axis == '+x':
            return cls(scale,0,0)
        elif axis == '-x':
            return cls(-scale,0,0)
        elif axis == 'y' or axis == '+y':
            return cls(0,scale,0)
        elif axis == '-y':
            return cls(0,-scale,0)
        elif axis == 'z' or axis == '+x':
            return cls(0,0,scale)
        elif axis == '-z':
            return cls(0,0,-scale)
        else:
            raise Exception('axis is not valid')
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

        super(Position,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(Position,self).deep_update()

        self.updated_attribute('x','update')
        self.updated_attribute('y','update')
        self.updated_attribute('z','update')

    def shallow_update(self):
        super(Position,self).shallow_update()

        self.updated_attribute('x','update')
        self.updated_attribute('y','update')
        self.updated_attribute('z','update')
