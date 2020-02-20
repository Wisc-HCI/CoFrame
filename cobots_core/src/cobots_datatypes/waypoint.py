from node import Node
from geometry import Position, Orientation


class Waypoint(Node):

    def __init__(self, position=Position(), orientation=Orientation(), joints=None, name='', uuid=None, parent=None):
        super(Node,self).__init__('waypoint',name,uuid,parent)
        self._position = position
        self._orientation = orientation
        self._joints = joints

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value
        self._position.parent = self
        if self._parent != None:
            self._parent.child_changed_event(["waypoint","position"])

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation = value
        self._orientation.parent = self
        if self._parent != None:
            self._parent.child_changed_event(["waypoint","orientation"])

    @property
    def joints(self):
        return self._joints

    @joints.setter
    def joints(self, value):
        self._joints = value
        if self._parent != None:
            self._parent.child_changed_event(["waypoint","joints"])

    def child_changed_event(self, attribute_trace):
        if self._parent != None:
            self._parent.child_changed_event(['waypoint'] + attribute_trace)

    def create_pose(self):
        return Pose(self.position,self.orientation,None)

    def to_dct(self):
        msg = super(Node,self).to_dct()
        msg.update({
            'position': self.position.to_dct(),
            'orientation': self.orientation.to_dct(),
            'joints': self.joints
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            position=Position.from_dct(dct['position']),
            orientation=Orientation.from_dct(dct['orientation']),
            name=dct['name'],
            uuid=dct['uuid'],
            joints=dct['joints'])
