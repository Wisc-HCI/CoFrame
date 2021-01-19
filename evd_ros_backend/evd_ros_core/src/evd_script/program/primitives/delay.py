from ..primitive import Primitive


class Delay(Primitive):

    '''
    Data structure methods
    '''
    def __init__(self, duration=0, type='', name='', uuid=None, parent=None, append_type=True):
        self._duration = None

        super(Delay,self).__init__(
            type='delay.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.duration = duration

    def to_dct(self):
        msg = super(Delay,self).to_dct()
        msg.update({
            'duration': self.duration,
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            type=dct['type'],
            append_type=False,
            uuid=dct['uuid'],
            duration=dct['duration'])

    '''
    Data accessor/modifier methods
    '''

    @property
    def duration(self):
        return self._duration

    @duration.setter
    def duration(self, value):
        if self._duration != value:
            self._duration = value
            self.updated_attribute("duration",'set')

    def set(self, dct):
        duration = dct.get('duration', None)
        if duration != None:
            self.duration = duration

        super(Delay,self).set(dct)

    '''
    Update Methods
    '''

    def deep_update(self):

        super(Delay,self).deep_update()

        self.updated_attribute('duration','update')

    def shallow_update(self):
        super(Delay,self).shallow_update()

        self.updated_attribute('duration','update')
