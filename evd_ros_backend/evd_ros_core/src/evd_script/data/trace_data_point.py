from .grade import Grade
from .geometry import Pose, Position, Orientation
from ..visualizable import VisualizeMarker, ColorTable

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class TraceDataPoint(Pose, VisualizeMarker):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls):
        return 'trace-data-point.'

    @classmethod
    def full_type_string(cls):
        return Pose.full_type_string() + cls.type_string()

    def __init__(self, position=None, orientation=None, grades={}, type='',
                 name='', uuid=None, parent=None, append_type=True):

        self._grades = {}

        super(TraceDataPoint,self).__init__(
            position=position,
            orientation=orientation,
            type='trace-data-point.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.grades = grades

    def to_dct(self):
        msg = super(TraceDataPoint,self).to_dct()
        msg.update({
            'grades': {key: self.grades[key].to_dct() for key in self.grades.keys()}
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
            grades={key: Grade.from_dct(dct['grades'][key]) for key in dct['grades'].keys()})

    def to_ros_marker(self, frame_id, id=0):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.SPHERE
        marker.ns = 'render_points'
        marker.id = id
        marker.pose = self.to_ros()
        marker.scale = Vector3(0.025,0.025,0.025)
        marker.color = ColorTable.TRACE_DATA_POINT_COLOR

        return marker

    '''
    Data accessor/modifier methods
    '''

    @property
    def grades(self):
        return self._grades

    @grades.setter
    def grades(self, value):
        if self._grades != value:

            for grade in self._grades.values():
                grade.remove_from_cache()
            
            self._grades = value
            for grade in self._grades.values():
                grade.parent = self

            self.updated_attribute('grades','set')

    def set(self, dct):

        if 'grades' in dct.keys():
            self.grades = {key: Grade.from_dct(dct['grades'][key]) for key in dct['grades'].keys()}

        super(TraceDataPoint,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(TraceDataPoint,self).deep_update()

        for grade in self.grades.values():
            grade.deep_update()

        self.updated_attribute('grades','update')

    def shallow_update(self):
        super(TraceDataPoint,self).shallow_update()

        self.updated_attribute('grades','update')