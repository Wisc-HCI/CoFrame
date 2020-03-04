from ..node import Node
from geometry import Pose, Position, Orientation


class TraceDataPoint(Pose):

    '''
    Data structure methods
    '''

    def __init__(self, position=None, orientation=None, grade=0, type='',
                 name='', uuid=None, parent=None, append_type=True):

        self._grade = None

        super(TraceDataPoint,self).__init__(
            position=position,
            orientation=orientation,
            type='trace.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.grade = grade

    def to_dct(self):
        msg = super(TraceDataPoint,self).to_dct()
        msg.update({
            'grade': self.grade
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            position=Position.from_dct(dct['position']),
            orientation=Orientation.from_dct(dct['orientation']),
            type=dct['type'],
            append_type=False,
            name=dct['name'],
            uuid=dct['uuid'],
            grade=dct['grade'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def grade(self):
        return self._grade

    @grade.setter
    def grade(self, value):
        if self._grade != value:
            self._grade = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('grade','set')])

    def set(self, dct):
        grade = dct.get('grade',None)
        if grade != None:
            self.grade = grade

        super(TraceDataPoint,self).set(dct)


class Trace(Node):

    '''
    Data structure methods
    '''

    def __init__(self, eePath=None, data={}, jPaths=[], tPaths=[], cPaths=[],
                 time=0, type='', name='', uuid=None, parent=None, append_type=True):

        self._data = {}
        self._time = None
        self._end_effector_path = None
        self._joint_paths = None
        self._tool_paths = None
        self._component_paths = None

        super(Trace,self).__init__(
            type='trace.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.data = data
        self.time = time
        self.end_effector_path = eePath
        self.joint_paths = jPaths
        self.tool_paths = tPaths
        self.component_paths = cPaths

    def to_dct(self):
        msg = super(Trace,self).to_dct()
        msg.update({
            'data': {key: [d.to_dct() for d in self.data[key]] for key in self.data.keys()},
            'time': self.time,
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
            data={key: [TraceDataPoint.from_dct(dct['data'][key][i]) for i in range(0,len(dct['data'][key]))] for key in dct['data'].keys()},
            eePath=dct['end_effector_path'],
            jPaths=dct['joint_paths'],
            tPaths=dct['tool_paths'],
            cPaths=dct['component_paths'],
            time=dct['time'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, value):
        if self._data != value:

            # remove old from cache
            for key in self._data.keys():
                for d in self._data[key]:
                    d.remove_from_cache()

            self._data = value
            for key in self._data.keys():
                for d in self._data[key]:
                    d.parent = self

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('data','set')])

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, value):
        if self._time != value:
            self._time = value

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('time','set')])

    @property
    def end_effector_path(self):
        return self._end_effector_path

    @end_effector_path.setter
    def end_effector_path(self, value):
        if self._end_effector_path != value:
            self._end_effector_path = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('end_effector_path','set')])

    @property
    def joint_paths(self):
        return self._joint_paths

    @joint_paths.setter
    def joint_paths(self, value):
        if self._joint_paths != value:
            self._joint_paths = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('joint_paths','set')])

    @property
    def tool_paths(self):
        return self._tool_paths

    @tool_paths.setter
    def tool_paths(self, value):
        if self._tool_paths != value:
            self._tool_paths = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('tool_paths','set')])

    @property
    def component_paths(self):
        return self._component_paths

    @component_paths.setter
    def component_paths(self, value):
        if self._component_paths != value:
            self._component_paths = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('component_paths','set')])

    def add_data_point(self, dp, group):
        dp.parent = self

        if not group in self._data.keys():
            self._data[group] = []
        self._data[group].append(dp)

        if self._parent != None:
            self._parent.child_changed_event(
                [self._child_changed_event_msg('data','add')])

    def get_data_point(self, uuid, group):
        for d in self._data[group]:
            if d.uuid == uuid:
                return d
        return None

    def delete_data_point(self, uuid, group):
        delIdx = None
        for i in range(0,len(self._data[group])):
            if self._data[group][i].uuid == uuid:
                delIdx = i
                break

        if delIdx != None:
            self._data[group].pop(delIdx).remove_from_cache()

            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('data','delete')])

    def set(self, dct):
        data = dct.get('data',None)
        if data != None:
            self.data = data

        time = dct.get('time',None)
        if time != None:
            self.time = time

        if 'end_effector_path' in dct.keys():
            self.end_effector_path = dct['end_effector_path']

        if 'joint_paths' in dct.keys():
            self.joint_paths = dct['joint_paths']

        if 'tool_paths' in dct.keys():
            self.tool_paths = dct['tool_paths']

        if 'component_paths' in dct.keys():
            self.component_paths = dct['component_paths']

        super(Trace,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for key in self._data.keys():
            for d in self._data[key]:
                d.remove_from_cache()
        super(Trace,self).remove_from_cache()

    def add_to_cache(self):
        for key in self._data.keys():
            for d in self._data[key]:
                d.add_to_cache()
        super(Trace,self).add_to_cache()

    '''
    Children methods
    '''

    def delete_child(self, uuid):

        group = None
        for g in self._data.keys():
            for d in self._data[g]:
                if d.uuid == uuid:
                    group = g
                    break

        if group != None:
            self.delete_data_point(uuid, group)
            return True
        else:
            return False
