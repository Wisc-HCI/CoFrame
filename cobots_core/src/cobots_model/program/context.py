from ..data.location import Location
from ..data.machine import Machine


class Context(object):

    def __init__(self, locations=[], machines=[], parent_context=None, parent_node=None):
        self._parent_context = parent_context
        self._parent_node = parent_node

        self._locations = {}
        for l in locations:
            l.parent = self._parent_node
            self._locations[l.uuid] = l

        self._machines = {}
        for m in machines:
            m.parent = self._parent_node
            self._machines[m.uuid] = m

    @property
    def locations(self):
        return self._locations.values()

    @property
    def machines(self):
        return self._machines.values()

    @property
    def parent_context(self):
        return self._parent_context

    @parent_context.setter
    def parent_context(self, value):
        if (self._parent_context != value):
            self._parent_context = value
            self._parent_node.child_changed_event(
                [self._parent_node._child_changed_event_msg('context_parent_context','set')])

    @property
    def parent_node(self):
        return self._parent_node

    @parent_node.setter
    def parent_node(self, value):
        if self._parent_node != value:
            self._parent_node = value
            self._parent_node.child_changed_event(
                [self._parent_node._child_changed_event_msg('context_parent_node','set')])

    def get_location(self, uuid):
        if uuid in self._locations.keys():
            return self._locations[uuid]
        else:
            return None

    def add_location(self, location):
        location.parent = self._parent_node
        self._locations[location.uuid] = location
        self._parent_node.cache.add(location.uuid,location)

        self._parent_node.child_changed_event(
            [self._parent_node._child_changed_event_msg('context_locations','add')])

    def delete_location(self, uuid):
        if uuid in self._locations.keys():
            self._locations.pop(uuid).remove_from_cache()
            self._parent_node.child_changed_event(
                [self._parent_node._child_changed_event_msg('context_locations','delete')])

    def get_machine(self, uuid):
        if uuid in self._machines.keys():
            return self._machines[uuid]
        else:
            return None

    def add_machine(self, machine):
        machine.parent = self._parent_node
        self._machines[machine.uuid] = machine
        self._parent_node.cache.add(machine.uuid,machine)

        self._parent_node.child_changed_event(
            [self._parent_node._child_changed_event_msg('context_machines','add')])

    def delete_machine(self, uuid):
        if uuid in self._machines.keys():
            self._machines.pop(uuid).remove_from_cache()
            self._parent_node.child_changed_event(
                [self._parent_node._child_changed_event_msg('context_machines','delete')])

    def to_dct(self):
        return {
            'locations': [l.to_dct() for l in self.locations],
            'machines': [m.to_dct() for m in self.machines]
        }

    @classmethod
    def from_dct(cls, dct):
        return Context(
            locations=[Location.from_dct(l) for l in dct['locations']],
            machines=[Machine.from_dct(m) for m in dct['machines']]
        )

    def remove_from_cache(self):
        for m in self._machines:
            m.remove_from_cache()

        for l in self._locations:
            l.remove_from_cache()
