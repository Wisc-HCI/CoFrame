import geometry_msgs.msg as ros_msgs

from node import Node


class Pose(Node):

    def __init__(self, position=None, orientation=None, parent=None):
        super(Node,self).__init__('pose', parent=parent)

        if position is None:
            self._position = Position(0,0,0,self)
        else:
            self._position = position
            self._position.parent = self

        if orientation is None:
            self._orientation = Orientation(0,0,0,1,self)
        else:
            self._orientation = orientation
            self._orientation.parent = self

    def child_changed_event(self, attribute_trace):
        if self._parent != None:
            self._parent.child_changed_event(['pose'] + attribute_trace)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value
        self._position.parent = self
        if self._parent != None:
            self._parent.child_changed_event(["pose","position"])

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation = value
        self._orientation.parent = self
        if self._parent != None:
            self._parent.child_changed_event(["pose","orientation"])

    def to_dct(self):
        msg = super(Node,self).to_dct()
        msg.update({
            'position': self.position.to_dct(),
            'orientation': self.orientation.to_dct()
        })
        return msg

    def to_ros(self):
        return ros_msgs.Pose(position=self.position.to_ros(),
                    orientation=self.orientation.to_ros())

    @classmethod
    def from_dct(self, dct):
        return cls(position=Position.from_dct(dct['position']),
                   orientation=Orientation.from_dct(dct['orientation']))

    @classmethod
    def from_ros(self, obj):
        return cls(position=Position.from_ros(obj.position),
                   orientation=Orientation.from_ros(obj.orientation))


class Position(Node):

    def __init__(self, x, y, z, parent=None):
        super(Node,self).__init__('position', parent=parent)
        self._x = x
        self._y = y
        self._z = z

    def child_changed_event(self, attribute_trace):
        if self._parent != None:
            self._parent.child_changed_event(['position'] + attribute_trace)

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value
        if self._parent != None:
            self._parent.child_changed_event(['position','x'])

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value
        if self._parent != None:
            self._parent.child_changed_event(['position','y'])

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        self._z = value
        if self._parent != None:
            self._parent.child_changed_event(['position','z'])

    def to_dct(self):
        msg = super(Node,self).to_dct()
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


class Orientation(Node):

    def __init__(self, x, y, z, w, parent=None):
        super(Node,self).__init__('orientation', parent=parent)
        self._x = x
        self._y = y
        self._z = z
        self._w = w
        self._parent = parent

    def child_changed_event(self, attribute_trace):
        if self._parent != None:
            self._parent.child_changed_event(['orientation'] + attribute_trace)

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value
        if self._parent != None:
            self._parent.child_changed_event(['orientation','x'])

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value
        if self._parent != None:
            self._parent.child_changed_event(['orientation','y'])

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        self._z = z
        if self._parent != None:
            self._parent.child_changed_event(['orientation','z'])

    @property
    def w(self):
        return self._w

    @w.setter
    def w(self, value):
        self._w = value
        if self._parent != None:
            self._parent.child_changed_event(['orientation','w'])

    def to_dct(self):
        msg = super(Node,self).to_dct()
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
