'''

'''

from node import *
from task import *
from cache import *
from trace import *
from context import *
from geometry import *
from location import *
from waypoint import *
from primitive import *
from trajectory import *


class Program(Task):

    def __init__(self, primitives=[], name='', type='', uuid=None, append_type=True, context=None):
        super(Program,self).__init__(
            type='program.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=None,
            append_type=append_type,
            primitives=primitives,
            context=context)

        self._cache = Cache()

    def child_changed_event(self, attribute_trace):
        pass #TODO all updates to tree pass through here

    @property
    def cache(self):
        return self._cache

    def get(self, field, uuid):
        pass
