import tf
import geometry_msgs.msg as ros_msgs

from ..node import Node


class Pose(Node):

    '''
    Data structure methods
    '''

    def __init__(self, position=None, orientation=None, type='', name='',
                 uuid=None, parent=None, append_type=True):

        self._position = None
        self._orientation = None

        super(Pose,self).__init__(
            type='pose.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        if position is None:
            self.position = Position(0,0,0)
        else:
            self.position = position

        if orientation is None:
            self.orientation = Orientation(0,0,0,1)
        else:
            self.orientation = orientation

    def to_dct(self):
        msg = super(Pose,self).to_dct()
        msg.update({
            'position': self.position.to_dct(),
            'orientation': self.orientation.to_dct()
        })
        return msg

    def to_ros(self):
        return ros_msgs.Pose(position=self.position.to_ros(),
                    orientation=self.orientation.to_ros())

    @classmethod
    def from_dct(cls, dct):
        return cls(position=Position.from_dct(dct['position']),
                   orientation=Orientation.from_dct(dct['orientation']),
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    @classmethod
    def from_ros(self, obj):
        return cls(position=Position.from_ros(obj.position),
                   orientation=Orientation.from_ros(obj.orientation))

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

    def set(self, dct):
        pos = dct.get('position',None)
        if pos != None:
            self.position = Position.from_dct(pos)

        ort = dct.get('orientation',None)
        if ort != None:
            self.orientation = Orientation.from_dct(ort)

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

    def deep_update(self):
        self.orientation.deep_update()
        self.position.deep_update()

        super(Pose,self).deep_update()

        self.updated_attribute('position','update')
        self.updated_attribute('orientation','update')

    def shallow_update(self):
        super(Pose,self).shallow_update()

        self.updated_attribute('position','update')
        self.updated_attribute('orientation','update')


class Position(Node):

    '''
    Data structure methods
    '''

    def __init__(self, x, y, z, type='', name='', parent=None, uuid=None, append_type=True):
        self._x = None
        self._y = None
        self._z = None

        super(Position,self).__init__(
            type='position.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

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
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')

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


class Orientation(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def Identity(cls):
        return cls(0,0,0,1)

    def __init__(self, x, y, z, w, type='', name='', uuid=None, parent=None, append_type=True):
        self._x = None
        self._y = None
        self._z = None
        self._w = None

        super(Orientation,self).__init__(
            type='orientation.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

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
            uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
            name=dct['name'] if 'name' in dct.keys() else '')

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

    def to_euler():
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
