from ..node import Node
from ..data.location import Location
from ..data.machine import Machine


class Context(Node):

    '''
    Data structure methods
    '''

    def __init__(self, locations=[], machines=[], parent_context=None, type='',
                 name='', uuid=None, parent=None, append_type=True):

        self._locations = {}
        self._machines = {}
        self._parent_context = None

        super(Context,self).__init__(
            type='context.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.parent_context = parent_context
        self.locations = locations
        self.machines = machines

    def to_dct(self):
        msg = super(Context,self).to_dct()
        msg.update({
            'locations': [l.to_dct() for l in self.locations],
            'machines': [m.to_dct() for m in self.machines]
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return Context(
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            locations=[Location.from_dct(l) for l in dct['locations']],
            machines=[Machine.from_dct(m) for m in dct['machines']]
        )

    '''
    Data accessor/modifier methods
    '''

    @property
    def locations(self):
        return self._locations.values()

    @locations.setter
    def locations(self, value):
        for l in self._locations:
            l.remove_from_cache()
        self._locations = {}

        for l in value:
            self._locations[l.uuid] = l
            l.parent = self

        self.updated_attribute('locations','set')

    @property
    def machines(self):
        return self._machines.values()

    @machines.setter
    def machines(self, value):
        for m in self._machines:
            m.remove_from_cache()
        self._machines = {}

        for m in value:
            self._machines[m.uuid] = m
            m.parent = self

        self.updated_attribute('machines','set')

    @property
    def parent_context(self):
        return self._parent_context

    @parent_context.setter
    def parent_context(self, value):
        if self._parent_context != value:
            self._parent_context = value
            self.updated_attribute('parent_context','set')

    def get_location(self, uuid):
        if uuid in self._locations.keys():
            return self._locations[uuid]
        else:
            return None

    def add_location(self, location):
        location.parent = self
        self._locations[location.uuid] = location
        self.updated_attribute('locations','add', location.uuid)

    def delete_location(self, uuid):
        if uuid in self._locations.keys():
            self._locations.pop(uuid).remove_from_cache()
            self.updated_attribute('locations','delete', uuid)

    def get_machine(self, uuid):
        if uuid in self._machines.keys():
            return self._machines[uuid]
        else:
            return None

    def add_machine(self, machine):
        machine.parent = self
        self._machines[machine.uuid] = machine
        self.updated_attribute('machines','add', machine.uuid)

    def delete_machine(self, uuid):
        if uuid in self._machines.keys():
            self._machines.pop(uuid).remove_from_cache()
            self.updated_attribute('machines','delete', uuid)

    def set(self, dct):

        if 'locations' in dct.keys():
            self.locations = [Location.from_dct(l) for l in dct['locations']]

        if 'machines' in dct.keys():
            self.machines = [Machine.from_dct(m) for m in dct['machines']]

        super(Context,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for m in self.machines:
            m.remove_from_cache()

        for l in self.locations:
            l.remove_from_cache()

        super(Context,self).remove_from_cache()

    def add_to_cache(self):
        for m in self.machines:
            m.add_to_cache()

        for l in self.locations:
            l.add_to_cache()

        super(Context,self).add_to_cache()

    '''
    Children methods
    '''

    def delete_child(self, uuid):
        success = False

        if uuid in [l.uuid for l in self.locations]:
            self.delete_location(uuid)
            success = True
        elif uuid in [m.uuid for m in self.machines]:
            self.delete_machine(uuid)
            success = True

        return success

    '''
    Update Methods
    '''

    def deep_update(self):

        for l in self.locations:
            l.deep_update()

        for m in self.machines:
            m.deep_update()

        super(Context,self).deep_update()

        self.updated_attribute('machines','update')
        self.updated_attribute('locations','update')

    def shallow_update(self):
        super(Context,self).shallow_update()

        self.updated_attribute('machines','update')
        self.updated_attribute('locations','update')
