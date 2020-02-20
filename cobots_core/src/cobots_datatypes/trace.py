from node import Node
from geometry import Position, Orientation


class TraceDataPoint(Node):

    def __init__(self, position=Position(), orientation=Orientation(), grade=0, uuid=None):
        super(Node,self).__init__('trace-data-point')
        self._position = position
        self._orientation = orientation
        self._grade = grade

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value
        self._position.parent = self
        if self._parent != None:
            self._parent.child_changed_event(["trace-data-point","position"])

    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation = value
        self._orientation.parent = self
        if self._parent != None:
            self._parent.child_changed_event(["trace-data-point","orientation"])

    @property
    def grade(self):
        return self._orientation

    @grade.setter
    def grade(self, value):
        self._grade = value
        if self._parent != None:
            self._parent.child_changed_event(["trace-data-point","grade"])

    def child_changed_event(self, attribute_trace):
        if self._parent != None:
            self._parent.child_changed_event(['trace-data-point'] + attribute_trace)

    def to_dct(self):
        msg = super(Node,self).to_dct()
        msg.update({
            'position': self.position.to_dct(),
            'orientation': self.orientation.to_dct(),
            'grade': self.grade
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            uuid=dct['uuid'],
            position=Position.from_dct(dct['position']),
            orientation=Orientation.from_dct(dct['orientation']),
            grade=dct['grade']
        )


class Trace(Node):

    def __init__(self, data, eePath, jPaths, tPaths, cPaths, uuid=None, parent=parent):
        super(Node,self).__init__('trace','',uuid, parent=parent)
        self._data = data
        self._end_effector_path = eePath
        self._joint_paths = jPaths
        self._tool_paths = tPaths
        self._component_paths = cPaths

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, value):
        self._data = value
        for i in range(0,len(self._data)):
            self._data[i].parent = self

        if self._parent != None:
            self._parent.child_changed_even(["trace","data"])

    @property
    def end_effector_path(self):
        return self._end_effector_path

    @end_effector_path.setter
    def end_effector_path(self, value):
        self._end_effector_path = value
        if self._parent != None:
            self._parent.child_changed_even(["trace","end_effector_path"])

    @property
    def joint_paths(self):
        return self._joint_paths

    @joint_paths.setter(self, value):
    def joint_paths(self, value):
        self._joint_paths = value
        if self._parent != None:
            self._parent.child_changed_even(["trace","joint_paths"])

    @property
    def tool_paths(self):
        return self._tool_paths

    @tool_paths.setter
    def tool_paths(self, value):
        self._tool_paths = value
        if self._parent != None:
            self._parent.child_changed_even(["trace","tool_paths"])

    @property
    def component_paths(self):
        return self._component_paths

    @component_paths.setter
    def component_paths(self, value):
        self._component_paths = value
        if self._parent != None:
            self._parent.child_changed_even(["trace","component_paths"])

    def to_dct(self):
        msg = super(Node,self).to_dct()
        msg.update({
            'data': [self.data[i].to_dct() for i in range(0,len(self.data))],
            'end_effector_path': self.end_effector_path,
            'joint_paths': self.joint_paths,
            'tool_paths': self.tool_paths,
            'component_paths': self.component_paths
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            uuid=dct['uuid'],
            data=[TraceDataPoint.from_dct(dct['data'][i]) for i in range(0,len(dct['data']))],
            eePath=dct['end_effector_path'],
            jPaths=dct['joint_paths'],
            tPaths=dct['tool_paths'],
            cPaths=dct['component_paths']
        )
