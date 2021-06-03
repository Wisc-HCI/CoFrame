'''
Trace is the result of running a trajectory on the robot planner.

Traces provide a set of data and various labeled keys.

data = {
    "key": [TraceDataPoint,...]
}

keys are provided by
- end_effector_path (robot's end-effector frame)
- joint_paths (each relevant joint frame)
- tool_paths (each relevant tool frame)
- component_paths (additional frames tracked by application)

time is the duration for just this trajectory in planning (approximate)
'''

from ..node import Node
from .geometry import Pose
from ..visualizable import VisualizeMarkers, ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class Trace(Node, VisualizeMarkers):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'trace' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, tf_data={}, time_data=[], joint_data=[], grades={},
                 eePath=None, jPaths=[], tPaths=[], cPaths=[], time=0, type='',
                 name='', uuid=None, parent=None, append_type=True, editable=True,
                 deleteable=True, description=''):

        super(Trace,self).__init__(
            type=Trace.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description='')

        self._time_data = time_data
        self._joint_data = joint_data
        self._tf_data = tf_data
        self._grades = grades
        self._time = time
        self._end_effector_path = eePath
        self._joint_paths = jPaths
        self._tool_paths = tPaths
        self._component_paths = cPaths

    def to_dct(self):
        msg = super(Trace,self).to_dct()
        msg.update({
            'time_data': self.time_data,
            'joint_data': self.joint_data,
            'tf_data': self.tf_data,
            'grades': self.grades,
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
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            time_data=dct['time_data'],
            joint_data=dct['joint_data'],
            tf_data=dct['tf_data'],
            grades=dct['grades'],
            eePath=dct['end_effector_path'],
            jPaths=dct['joint_paths'],
            tPaths=dct['tool_paths'],
            cPaths=dct['component_paths'],
            time=dct['time'])

    def to_ros_markers(self, frame_id, id_start=0):
        render_point_markers = []
        line_markers = []
        render_point_uuids = []

        count = id_start
        for key in self.tf_data.keys():
            lineMarker = Marker()
            lineMarker.header.frame_id = frame_id
            lineMarker.type = Marker.LINE_STRIP
            lineMarker.ns = 'traces'
            lineMarker.id = count
            lineMarker.scale = Vector3(0.01,0.01,0.01)

            count += 1

            pointsList = []
            uuidsList = []
            for point in self.tf_data[key]:
                marker = Pose.from_simple_dct(point).to_ros_marker(frame_id,count)
                lineMarker.points.append(marker.pose.position)
                pointsList.append(marker)
                uuidsList.append(point.uuid)
                count += 1

            # trace path color based on group
            if key == self.end_effector_path:
                lineMarker.color = ColorTable.TRACE_END_EFFECTOR_COLOR
            elif key in self.joint_paths:
                lineMarker.color = ColorTable.TRACE_JOINT_COLOR
            elif key in self.tool_paths:
                lineMarker.color = ColorTable.TRACE_TOOL_COLOR
            else:
                lineMarker.color = ColorTable.TRACE_COMPONENT_COLOR

            render_point_markers.append(pointsList)
            line_markers.append(lineMarker)
            render_point_uuids.append(uuidsList)

        return line_markers, render_point_markers, render_point_uuids


    '''
    Data accessor/modifier methods
    '''

    @property
    def time_data(self):
        return self._time_data

    @property
    def joint_data(self):
        return self._joint_data

    @property
    def tf_data(self):
        return self._tf_data

    @property
    def grades(self):
        return self._grades

    @property
    def time(self):
        return self._time

    @property
    def end_effector_path(self):
        return self._end_effector_path

    @property
    def joint_paths(self):
        return self._joint_paths

    @property
    def tool_paths(self):
        return self._tool_paths

    @property
    def component_paths(self):
        return self._component_paths

    '''
    Update Methods
    '''

    def deep_update(self):
        super(Trace,self).deep_update()

        self.updated_attribute('tf_data','update')
        self.updated_attribute('time','update')
        self.updated_attribute('end_effector_path','update')
        self.updated_attribute('joint_paths','update')
        self.updated_attribute('tool_paths','update')
        self.updated_attribute('component_paths','update')

    def shallow_update(self):
        super(Trace,self).shallow_update()

        self.updated_attribute('tf_data','update')
        self.updated_attribute('time','update')
        self.updated_attribute('end_effector_path','update')
        self.updated_attribute('joint_paths','update')
        self.updated_attribute('tool_paths','update')
        self.updated_attribute('component_paths','update')
