from abstract import Abstract
from geometry import Position, Orientation


class TraceDataPoint(Abstract):

    def __init__(self, position=Position(), orientation=Orientation(), grade, uuid=None):
        Abstract.__init__(self,'trace_data_point','',uuid)
        self.position = position
        self.orientation = orientation
        self.grade = grade

    def to_dct(self):
        return {
            'uuid': self.uuid,
            'type': self.type,
            'name': self.name,
            'position': self.position.to_dct(),
            'orientation': self.orientation.to_dct(),
            'grade': self.grade
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            uuid=dct['uuid'],
            position=Position.from_dct(dct['position']),
            orientation=Orientation.from_dct(dct['orientation']),
            grade=dct['grade']
        )


class TraceTrajectory(Abstract):

    def __init__(self, data, eePath, jPaths, tPaths, cPaths, uuid=None):
        Abstract.__init__(self,'trace_trajectory','',uuid)
        self.data = data
        self.end_effector_path = eePath
        self.joint_paths = jPaths
        self.tool_paths = tPaths
        self.component_paths = cPaths

    def to_dct(self):
        return {
            'uuid': self.uuid,
            'type': self.type,
            'name': self.name,
            'data': [self.data[i].to_dct(), for i in range(0,len(self.data))]
            'end_effector_path': self.end_effector_path,
            'joint_paths': self.joint_paths,
            'tool_paths': self.tool_paths,
            'component_paths': self.component_paths
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            uuid=dct['uuid'],
            data=[TraceDataPoint.from_dct(dct['data'][i]) for i in range(0,len(dct['data']))],
            eePath=dct['end_effector_path'],
            jPaths=dct['joint_paths']
            tPaths=dct['tool_paths']
            cPaths=dct['component_paths']
        )
