'''
Simple primitive that delays the cobot's behavior for a fixed amount of time.
'''

import time

from ..primitive import Primitive
from ... import NUMBER_TYPE


class Delay(Primitive):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Delay'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'delay' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Primitive.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = Primitive.template()
        template['parameters'].append({
            'type': NUMBER_TYPE,
            'key': 'duration',
            'is_uuid': False,
            'is_list': False
        })
        return template

    def __init__(self, duration=0, parameters=None, type='', name='', uuid=None, parent=None,
                 append_type=True, editable=True, deleteable=True, description=''):

        if parameters == None:
            parameters = {
                'duration': duration
            }

        super(Delay,self).__init__(
            type=Delay.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description,
            parameters=parameters)

    '''
    Data accessor/modifier methods
    '''

    @property
    def duration(self):
        return self._parameters['duration']

    @duration.setter
    def duration(self, value):
        if self._parameters['duration'] != value:
            self._parameters['duration'] = value
            self.updated_attribute("parameters.duration",'set')

    def set(self, dct):
        duration = dct.get('duration', None)
        if duration != None:
            self.duration = duration

        super(Delay,self).set(dct)

    '''
    Execution methods
    (uses default node behavior for symbolic)
    '''

    def realtime_execution(self, hooks):
        hooks.active_primitive = self

        # initialize state
        if not self.uuid in hooks.state.keys():
            hooks.state[self.uuid] = {'start_time': time.time(), 'paused_start_time': [], 'paused_end_time': [], 'in_pause': False}

        # handle timing for when program is paused
        if hooks.pause and not hooks.state[self.uuid]['in_pause']:
            hooks.state[self.uuid]['paused_start_time'].append(time.time())
        elif not hooks.pause and hooks.state[self.uuid]['in_pause']:
            hooks.state[self.uuid]['paused_end_time'].append(time.time())

        # enforce duration
        if not hooks.pause:
            total = time.time() - hooks.state[self.uuid]['start_time']

            for i in range(0,len(hooks.state[self.uuid]['paused_start_time'])):
                total -= hooks.state[self.uuid]['paused_end_time'][i] - hooks.state[self.uuid]['paused_start_time'][i]

            if total >= self.duration:
                next = self.parent
                del hooks.state[self.uuid]

        return next
