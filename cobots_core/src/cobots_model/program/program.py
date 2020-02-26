'''

'''

from task import *
from cache import *


class Program(Task):

    def __init__(self, primitives=[], server_cb=None, name='', type='', uuid=None, append_type=True, context=None):
        super(Program,self).__init__(
            type='program.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=None,
            append_type=append_type,
            primitives=primitives,
            context=context)

        self.server_cb = server_cb
        self._cache = Cache()
        self.add_to_cache(self.uuid,self)

    def child_changed_event(self, attribute_trace):
        if self.server_cb != None:
            self.server_cb(attribute_trace)

    @property
    def cache(self):
        return self._cache

    def set(self, dct):
        pass #TODO write this

    def delete(self, uuid):
        obj = self._cache.get(uuid)
        obj.parent.delete_child(uuid)

    def create(self, field, dct):
        if field == 'location':
            pass #TODO
        elif field == 'machine':
            pass #TODO
        elif field == 'trajectory':
            pass #TODO
        elif field == 'waypoint':
            pass #TODO
        elif field == 'primitive':
            pass #TODO
        else:
            raise Exception('Field ({}) is not able to be created')
