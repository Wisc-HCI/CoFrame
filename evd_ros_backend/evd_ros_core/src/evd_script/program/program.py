'''

'''

from .task import Task
from ..environment import Environment


class Program(Task):

    '''
    Data structure methods
    '''

    def __init__(self, primitives=[], changes_cb=None, name='', type='', uuid=None, append_type=True, environment=None, context=None):
        self.changes_cb = changes_cb

        if environment != None and context != None:
            raise Exception("Environment is an alias for Context")
        elif environment == None and context == None:
            environment = Environment()
        elif context != None:
            environment = context

        if not isinstance(environment,Environment):
            raise Exception('Program level context must be an environment')

        super(Program,self).__init__(
            type='program.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=None,
            append_type=append_type,
            primitives=primitives,
            context=environment)

    '''
    Data accessor/modifier methods
    '''

    @property
    def environment(self):
        return self._context

    @environment.setter
    def environment(self, value):
        if self._context != value:
            if not isinstance(value,Environment):
                raise Exception('Program level context must be an environment')

            if self._context != None:
                self._context.remove_from_cache()

            self._context = value
            self._context.parent = self

            self.updated_attribute('context','set')

    '''
    Utility methods
    '''

    def child_changed_event(self, attribute_trace):
        if self.changes_cb != None:
            attribute_trace.append(self._child_changed_event_msg(None, 'callback'))
            self.changes_cb(attribute_trace)

    def updated_attribute(self, attribute, verb, child_uuid = None):
        event = [self._child_changed_event_msg(attribute, verb, child_uuid)]
        if self.changes_cb != None:
            self.changes_cb(event)
