from geometry import Pose, Position, Orientation


class Waypoint(Pose):

    '''
    Data structure methods
    '''

    def __init__(self, position=None, orientation=None, joints=None, type='',
                 name='', uuid=None, parent=None, append_type=True):
        super(Waypoint,self).__init__(
            position=position,
            orientation=orientation,
            type='waypoint.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self._joints = None
        self.joints = joints

    def to_dct(self):
        msg = super(Waypoint,self).to_dct()
        msg.update({
            'joints': self.joints
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
            joints=dct['joints'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def joints(self):
        return self._joints

    @joints.setter
    def joints(self, value):
        if self._joints != value:
            self._joints = value
            if self._parent != None:
                self._parent.child_changed_event(
                    [self._child_changed_event_msg('joints','set')])

    def set(self, dct):
        joints = dct.get('joints',None)
        if joints != None:
            self.joints = joints

        super(Waypoint,self).set(dct)
