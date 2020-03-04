'''

'''

from task import *
from cache import *


class Program(Task):

    '''
    Data structure methods
    '''

    def __init__(self, primitives=[], changes_cb=None, name='', type='', uuid=None, append_type=True, context=None):
        self.changes_cb = changes_cb
        self._cache = Cache()

        super(Program,self).__init__(
            type='program.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=None,
            append_type=append_type,
            primitives=primitives,
            context=context)

    '''
    Data accessor/modifier methods
    '''

    @property
    def cache(self):
        return self._cache

    '''
    Utility methods
    '''

    def child_changed_event(self, attribute_trace):
        if self.changes_cb != None:
            self.changes_cb(attribute_trace)
