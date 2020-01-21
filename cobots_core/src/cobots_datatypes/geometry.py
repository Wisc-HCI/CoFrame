import geometry_msgs.msg as ros_msgs


class Pose(object):

    def __init__(self, position=None, orientation=None):
        if position is None:
            self.position = Position(0,0,0)
        else:
            self.position = position

        if orientation is None:
            pass
        else:
            self.orientation = orientation

    def to_dct(self):
        return {
            'position': self.position.to_dct(),
            'orientation': self.orientation.to_dct()
        }

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


class Position(object):

    def __init__(self, x, y, z):
        self.x = x,
        self.y = y,
        self.z = z,

    def to_dct(self):
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


class Orientation(object):

    def __init__(self, x, y, z, w):
        self.x = x,
        self.y = y,
        self.z = z,
        self.w = w

    def to_dct(self):
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


class TraceDataPoint(object):

    def __init__(self, position, orientation, grade, uuid):
        self.position = position
        self.orienation = orienation
        self.grade = grade
        self.uuid = uuid

    @classmethod
    def from_dct(self, dct):
        return TraceDataPoint(
            Position.from_dct(dct['position']),
            Orientation.from_dct(dct['orienation']),
            dct['grade'])

    def to_dct(self):
        return {
            'position': self.position,
            'orienation': self.orienation,
            'grade': self.grade
        }
