import geometry_msgs.msg as ros_msgs

from node import Node


class Pose(Node):

    def __init__(self, position=None, orientation=None, type='', name='',
                 uuid=None, parent=None, append_type=True):
        super(Node,self).__init__(
            type='pose.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        if position is None:
            self.position = Position(0,0,0,parent=self)
        else:
            self.position = position

        if orientation is None:
            self.orientation = Orientation(0,0,0,1,parent=self)
        else:
            self.orientation = orientation

    def _initialize_private_members(self):
        self._position = None
        self._orientation = None

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        if (self._position != value):
            self._position = value
            self._position.parent = self
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('position')])

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if self._orientation != value:
            self._orientation = value
            self._orientation.parent = self
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('orientation')])

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
                   orientation=Orientation.from_dct(dct['orientation']),
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    @classmethod
    def from_ros(self, obj):
        return cls(position=Position.from_ros(obj.position),
                   orientation=Orientation.from_ros(obj.orientation))


class Position(Node):

    def __init__(self, x, y, z, type='', name='', parent=None, uuid=None, append_type=True):
        super(Node,self).__init__(
            type='position.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        self.x = x
        self.y = y
        self.z = z

    def _initialize_private_members(self):
        self._x = None
        self._y = None
        self._z = None

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        if self._x != value:
            self._x = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('x')])

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        if self._y != value:
            self._y = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('y')])

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        if self._z != value:
            self._z = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('z')])

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
            z=dct['z'],
            type=dct['type'] if 'type' in dct.keys() else ''
            append_type=not 'type' in dct.keys()
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


class Orientation(Node):

    def __init__(self, x, y, z, w, type='', name='', uuid=None, parent=None, append_type=True):
        super(Node,self).__init__(
            type='orientation.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def _initialize_private_members(self):
        self._x = None
        self._y = None
        self._z = None
        self._w = None

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        if self._x != value:
            self._x = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('x')])

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        if self._y != value:
            self._y = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('y')])

    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        if self._z != value:
            self._z = z
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('z')])

    @property
    def w(self):
        return self._w

    @w.setter
    def w(self, value):
        if self._w != value:
            self._w = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('w')])

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
            w=dct['w'],
            type=dct['type'] if 'type' in dct.keys() else ''
            append_type=not 'type' in dct.keys()
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
