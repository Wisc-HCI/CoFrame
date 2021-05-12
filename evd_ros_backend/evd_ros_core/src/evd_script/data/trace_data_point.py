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

    def __init__(self, position=None, orientation=None, grade=0, type='',
                 name='', uuid=None, parent=None, append_type=True):

        self._grade = None

        super(TraceDataPoint,self).__init__(
            position=position,
            orientation=orientation,
            type='trace-data-point.'+type if append_type else type,
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
    def grade(self):
        return self._grade

    @grade.setter
    def grade(self, value):
        if self._grade != value:
            self._grade = value
            self.updated_attribute('grade','set')

    def set(self, dct):
        grade = dct.get('grade',None)
        if grade != None:
            self.grade = grade

        super(TraceDataPoint,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):
        super(TraceDataPoint,self).deep_update()

        self.updated_attribute('grade','update')

    def shallow_update(self):
        super(TraceDataPoint,self).shallow_update()

        self.updated_attribute('grade','update')
