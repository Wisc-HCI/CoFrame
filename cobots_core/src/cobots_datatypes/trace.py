from node import Node
from geometry import Pose, Position, Orientation


class TraceDataPoint(Pose):

    def __init__(self, position=None, orientation=None, grade=0, type='',
                 name='', uuid=None, parent=None, append_type=True):
        super(Pose,self).__init__(
            position=position,
            orientation=orientation,
            type='trace.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        self.grade = grade

    def _initialize_private_members(self):
        self._grade = None

    @property
    def grade(self):
        return self._orientation

    @grade.setter
    def grade(self, value):
        if self._grade != value:
            self._grade = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('grade')])

    def to_dct(self):
        msg = super(Pose,self).to_dct()
        msg.update({
            'grade': self.grade
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            position=Position.from_dct(dct['position']),
            orientation=Orientation.from_dct(dct['orientation']),
            type=dct['type']
            append_type=False,
            name=dct['name'],
            uuid=dct['uuid'],
            grade=dct['grade'])


class Trace(Node):

    def __init__(self, eePath=None, data=[], jPaths=[], tPaths=[], cPaths[], type='', name='', uuid=None, parent=None, append_type=True):
        super(Node,self).__init__(
            type='trace.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._initialize_private_members()

        self.data = data
        self.end_effector_path = eePath
        self.joint_paths = jPaths
        self.tool_paths = tPaths
        self.component_paths = cPaths

    def _initialize_private_members(self):
        self._data = None
        self._end_effector_path = None
        self._joint_paths = None
        self._tool_paths = None
        self._component_paths = None

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, value):
        if self._data != value:
            self._data = value
            for i in range(0,len(self._data)):
                self._data[i].parent = self

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('data')])

    @property
    def end_effector_path(self):
        return self._end_effector_path

    @end_effector_path.setter
    def end_effector_path(self, value):
        if self._end_effector_path != value:
            self._end_effector_path = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('end_effector_path')])
    @property
    def joint_paths(self):
        return self._joint_paths

    @joint_paths.setter(self, value):
    def joint_paths(self, value):
        if self._joint_paths != value:
            self._joint_paths = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('joint_paths')])
    @property
    def tool_paths(self):
        return self._tool_paths

    @tool_paths.setter
    def tool_paths(self, value):
        if self._tool_paths != value:
            self._tool_paths = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('tool_paths')])

    @property
    def component_paths(self):
        return self._component_paths

    @component_paths.setter
    def component_paths(self, value):
        if self._component_paths != value:
            self._component_paths = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('component_paths')])

    def add_data_point(self, dp):
        dp.parent = self
        self._data.append(dp)
        if self._parent != None:
            self._parent.child_changed_event(
                [self._child_changed_event_msg('data')])

    def get_data_point(self, uuid):
        for d in self._data:
            if d.uuid == uuid:
                return d
        return None

    def delete_data_point(self, uuid):
        delIdx = None
        for i in range(0,len(self._data)):
            if self._data[i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._data.pop(delIdx)
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('data')])

    def to_dct(self):
        msg = super(Node,self).to_dct()
        msg.update({
            'data': [d.to_dct() for d in self.data],
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
            type=dct['type'],
            name=dct['name'],
            append_type=False,
            data=[TraceDataPoint.from_dct(dct['data'][i]) for i in range(0,len(dct['data']))],
            eePath=dct['end_effector_path'],
            jPaths=dct['joint_paths'],
            tPaths=dct['tool_paths'],
            cPaths=dct['component_paths'])
