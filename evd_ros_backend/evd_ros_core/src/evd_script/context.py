
from .node import Node
from .data.thing import Thing
from .data.machine import Machine
from .data.waypoint import Waypoint
from .data.location import Location
from .data.trajectory import Trajectory

from .orphans import *


class Context(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls):
        return 'context.'

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    def __init__(self, locations=[], machines=[], things=[], waypoints=[], trajectories=[],
                 type='', name='', uuid=None, parent=None, append_type=True):

        self._orphan_list = evd_orphan_list()

        self._locations = {}
        self._machines = {}
        self._things = {}
        self._waypoints = {}
        self._trajectories = {}

        super(Context,self).__init__(
            type='context.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type)

        self.locations = locations
        self.machines = machines
        self.things = things
        self.waypoints = waypoints
        self.trajectories = trajectories

    def to_dct(self):
        msg = super(Context,self).to_dct()
        msg.update({
            'locations': [l.to_dct() for l in self.locations],
            'machines': [m.to_dct() for m in self.machines],
            'things': [t.to_dct() for t in self.things],
            'waypoints': [w.to_dct() for w in self.waypoints],
            'trajectories': [t.to_dct() for t in self.trajectories]
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
            machines=[Machine.from_dct(m) for m in dct['machines']],
            things=[Thing.from_dct(t) for t in dct['things']],
            waypoints=[Waypoint.from_dct(w) for w in dct['waypoints']],
            trajectories=[Trajectory.from_dct(t) for t in dct['trajectories']])

    def on_delete(self):

        for l in self.locations:
            self._orphan_list.add(l.uuid,'location')

        for m in self.machines:
            self._orphan_list.add(m.uuid,'machine')

        for t in self.things:
            self._orphan_list.add(t.uuid,'thing')

        for w in self.waypoints:
            self._orphan_list.add(w.uuid,'waypoint')

        for t in self.trajectories:
            self._orphan_list.add(t.uuid,'trajectory')

        super(Context,self).on_delete()

    '''
    Data accessor/modifier methods
    '''

    @property
    def context(self):
        return self

    @property
    def locations(self):
        return self._locations.values()

    @locations.setter
    def locations(self, value):
        uuids = []

        for l in self._locations:
            l.remove_from_cache()
            uuids.append(l.uuid)
        self._locations = {}

        for l in value:
            self._locations[l.uuid] = l
            l.parent = self
            if l.uuid in uuids:
                uuids.remove(l.uuid)

        for u in uuids:
            self._orphan_list.add(u,'location')

        self.updated_attribute('locations','set')

    @property
    def machines(self):
        return self._machines.values()

    @machines.setter
    def machines(self, value):
        uuids = []

        for m in self._machines:
            m.remove_from_cache()
            uuids.append(m.uuid)
        self._machines = {}

        for m in value:
            self._machines[m.uuid] = m
            m.parent = self
            if m.uuid in uuids:
                uuids.remove(m.uuid)

        for u in uuids:
            self._orphan_list.add(u,'machine')

        self.updated_attribute('machines','set')

    @property
    def things(self):
        return self._things.values()

    @things.setter
    def things(self, value):
        uuids = []

        for t in self._things:
            t.remove_from_cache()
            uuids.append(t.uuid)
        self._things = {}

        for t in value:
            self._things[t.uuid] = t
            t.parent = self
            if t.uuid in uuids:
                uuids.remove(t.uuid)

        for u in uuids:
            self._orphan_list.add(u,'thing')

        self.updated_attribute('things','set')

    @property
    def waypoints(self):
        return self._waypoints.values()

    @waypoints.setter
    def waypoints(self, value):
        uuids = []

        for w in self._waypoints:
            w.remove_from_cache()
            uuids.append(w.uuid)

        self._waypoints = {}

        for w in value:
            self._waypoints[w.uuid] = w
            w.parent = self
            if w.uuid in uuids:
                uuids.remove(w.uuid)

        for u in uuids:
            self._orphan_list.add(u,'waypoint')

        self.updated_attribute('waypoints','set')

    @property
    def trajectories(self):
        return self._trajectories.values()

    @trajectories.setter
    def trajectories(self, value):
        uuids = []

        for t in self._trajectories.values():
            t.remove_from_cache()
            uuids.append(t.uuid)

        self._trajectories = {}

        for t in value:
            self._trajectories[t.uuid] = t
            t.parent = self
            if t.uuid in uuids:
                uuids.remove(t.uuid)

        for u in uuids:
            self._orphan_list.add(u,'waypoint')

        self.updated_attribute('trajectories','set')

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
            self._orphan_list.add(uuid,'location')
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
            self._orphan_list.add(uuid,'machine')
            self.updated_attribute('machines','delete', uuid)

    def get_thing(self, uuid):
        if uuid in self._things.keys():
            return self._things[uuid]
        else:
            return None

    def add_thing(self, thing):
        thing.parent = self
        self._things[thing.uuid] = thing
        self.updated_attribute('things','add',thing.uuid)

    def delete_thing(self, uuid):
        if uuid in self._things.keys():
            self._things.pop(uuid).remove_from_cache()
            self._orphan_list.add(uuid,'thing')
            self.updated_attribute('things','delete', uuid)

    def get_waypoint(self, uuid):
        if uuid in self._waypoints.keys():
            return self._waypoints[uuid]
        else:
            return None

    def add_waypoint(self, waypoint):
        waypoint.parent = self
        self._waypoints[waypoint.uuid] = waypoint
        self.updated_attribute('waypoints','add',waypoint.uuid)

    def delete_waypoint(self, uuid):
        if uuid in self._waypoints.keys():
            self._waypoints.pop(uuid).remove_from_cache()
            self._orphan_list.add(uuid,'waypoint')
            self.updated_attribute('waypoints','delete',uuid)

    def get_trajectory(self, uuid):
        if uuid in self._trajectories.keys():
            return self._trajectories[uuid]
        else:
            return None

    def add_trajectory(self, trajectory):
        trajectory.parent = self
        self._trajectories[trajectory.uuid] = trajectory
        self.updated_attribute('trajectories','add',trajectory.uuid)

    def delete_trajectory(self, uuid):
        if uuid in self._trajectories.keys():
            self._trajectories.pop(uuid).remove_from_cache()
            self._orphan_list.add(uuid,'trajectory')
            self.updated_attribute('trajectories','delete',uuid)

    def set(self, dct):

        if 'locations' in dct.keys():
            self.locations = [Location.from_dct(l) for l in dct['locations']]

        if 'machines' in dct.keys():
            self.machines = [Machine.from_dct(m) for m in dct['machines']]

        if 'things' in dct.keys():
            self.things = [Thing.from_dct(t) for t in dct['things']]

        if 'waypoints' in dct.keys():
            self.waypoints = [Waypoint.from_dct(w) for w in dct['waypoints']]

        if 'trajectory' in dct.keys():
            self.trajectories = [Trajectory.from_dct(t) for t in dct['trajectories']]

        super(Context,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        for m in self.machines:
            m.remove_from_cache()

        for l in self.locations:
            l.remove_from_cache()

        for t in self.things:
            t.remove_from_cache()

        for w in self.waypoints:
            w.remove_from_cache()

        for t in self.trajectories:
            t.remove_from_cache()

        super(Context,self).remove_from_cache()

    def add_to_cache(self):
        for m in self.machines:
            m.add_to_cache()

        for l in self.locations:
            l.add_to_cache()

        for t in self.things:
            t.add_to_cache()

        for w in self.waypoints:
            w.add_to_cache()

        for t in self.trajectories:
            t.add_to_cache()

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
        elif uuid in [t.uuid for t in self.things]:
            self.delete_thing(uuid)
            success = True
        elif uuid in [w.uuid for w in self.waypoints]:
            self.delete_waypoint(uuid)
        elif uuid in [t.uuid for t in self.trajectories]:
            self.delete_trajectory(uuid)

        return success

    '''
    Update Methods
    '''

    def late_construct_update(self):

        for l in self.locations:
            l.late_construct_update()

        for m in self.machines:
            m.late_construct_update()

        for t in self.things:
            t.late_construct_update()

        for w in self.waypoints:
            w.late_construct_update()

        for t in self.trajectories:
            t.late_construct_update()

        super(Context,self).late_construct_update()

    def deep_update(self):

        for l in self.locations:
            l.deep_update()

        for m in self.machines:
            m.deep_update()

        for t in self.things:
            t.deep_update()

        for w in self.waypoints:
            w.deep_update()

        for t in self.trajectories:
            t.deep_update()

        super(Context,self).deep_update()

        self.updated_attribute('machines','update')
        self.updated_attribute('locations','update')
        self.updated_attribute('things','update')
        self.updated_attribute('waypoints','update')
        self.updated_attribute('trajectory','update')

    def shallow_update(self):
        super(Context,self).shallow_update()

        self.updated_attribute('machines','update')
        self.updated_attribute('locations','update')
        self.updated_attribute('things','update')
        self.updated_attribute('waypoints','update')
        self.updated_attribute('trajectory','update')
