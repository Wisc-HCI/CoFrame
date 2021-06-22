'''
Contexts at as a local "cache" for data within the program. At this point, EvD
is architected to have a single context at the root program though in theory this
could be used elsewhere.

Nodes that reference data should do so through the context if they do not want to
own the data with encapsulation.
'''

from .node import Node
from .data_nodes.thing import Thing
from .data_nodes.machine import Machine
from .data_nodes.waypoint import Waypoint
from .data_nodes.location import Location
from .data_nodes.thing_type import ThingType
from .data_nodes.grade_type import GradeType
from .data_nodes.trajectory import Trajectory
from .data_nodes.placeholder import Placeholder
from .type_defs import ALL_REGION_TYPES

from .orphans import *
from .node_parser import NodeParser


class Context(Node):

    '''
    Data structure methods
    '''

    @classmethod
    def display_name(cls):
        return 'Context'

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'context' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Node.full_type_string() + cls.type_string()

    @classmethod
    def template(cls):
        template = EnvironmentNode.template()
        template['fields'].append({
            'type': Location.full_type_string(),
            'key': 'locations',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': Machine.full_type_string(),
            'key': 'machines',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': Thing.full_type_string(),
            'key': 'things',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': ThingType.full_type_string(),
            'key': 'thing_types',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': Waypoint.full_type_string(),
            'key': 'waypoints',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': Trajectory.full_type_string(),
            'key': 'trajectories',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': ALL_REGION_TYPES,
            'key': 'regions',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': GradeType.full_type_string(),
            'key': 'grade_types',
            'is_uuid': False,
            'is_list': True
        })
        template['fields'].append({
            'type': Placeholder.full_type_string(),
            'key': 'placeholders',
            'is_uuid': False,
            'is_list': True
        })
        return template

    def __init__(self, locations=[], machines=[], things=[], thing_types=[],
                 waypoints=[], trajectories=[], regions=[], grade_types=[],
                 placeholders=[], type='', name='', uuid=None, parent=None, 
                 append_type=True, editable=True, deleteable=True, description=''):

        self._orphan_list = evd_orphan_list()

        self._locations = {}
        self._machines = {}
        self._things = {}
        self._thing_types = {}
        self._waypoints = {}
        self._trajectories = {}
        self._regions = {}
        self._grade_types = {}
        self._placeholders = {}

        super(Context,self).__init__(
            type=Context.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable,
            description=description)

        self.locations = locations
        self.machines = machines
        self.things = things
        self.waypoints = waypoints
        self.trajectories = trajectories
        self.thing_types = thing_types
        self.regions = regions
        self.grade_types = grade_types
        self.placeholders = placeholders

    def to_dct(self):
        msg = super(Context,self).to_dct()
        msg.update({
            'locations': [l.to_dct() for l in self.locations],
            'machines': [m.to_dct() for m in self.machines],
            'things': [t.to_dct() for t in self.things],
            'waypoints': [w.to_dct() for w in self.waypoints],
            'trajectories': [t.to_dct() for t in self.trajectories],
            'thing_types': [t.to_dct() for t in self.thing_types],
            'regions': [r.to_dct() for r in self.regions],
            'grade_types': [g.to_dct() for g in self.grade_types],
            'placeholders': [p.to_dct() for p in self.placeholders]
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return Context(
            name=dct['name'],
            uuid=dct['uuid'],
            type=dct['type'],
            append_type=False,
            editable=dct['editable'],
            deleteable=dct['deleteable'],
            description=dct['description'],
            locations=[NodeParser(l, enforce_types=[Location.type_string(trailing_delim=False)]) for l in dct['locations']],
            machines=[NodeParser(m, enforce_types=[Machine.type_string(trailing_delim=False)]) for m in dct['machines']],
            things=[NodeParser(t, enforce_types=[Thing.type_string(trailing_delim=False)]) for t in dct['things']],
            waypoints=[NodeParser(w, enforce_types=[Waypoint.type_string(trailing_delim=False)]) for w in dct['waypoints']],
            trajectories=[NodeParser(t, enforce_types=[Trajectory.type_string(trailing_delim=False)]) for t in dct['trajectories']],
            thing_types=[NodeParser(t, enforce_types=[ThingType.type_string(trailing_delim=False)]) for t in dct['thing_types']],
            regions=[NodeParser(r, enforce_types=[ALL_REGION_TYPES]) for r in dct['regions']],
            grade_types=[NodeParser(g, enforce_types=[GradeType.type_string(trailing_delim=False)]) for g in dct['grade_types']],
            placeholders=[NodeParser(p, enforce_types=[Placeholder.type_string(trailing_delim=False)]) for p in dct['placeholders']])

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

        for t in self.thing_types:
            self._orphan_list.add(t.uuid,'thing_type')

        for r in self.regions:
            self._orphan_list.add(r.uuid,'region')

        for g in self.grade_types:
            self._orphan_list.add(g.uuid,'grade_type')

        for p in self.placeholders:
            self._orphan_list.add(p.uuid,'placeholder')

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

        for l in self._locations.values():
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

        for m in self._machines.values():
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

        for t in self._things.values():
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
    def thing_types(self):
        return self._thing_types.values()

    @thing_types.setter
    def thing_types(self, value):
        uuids = []

        for t in self._thing_types.values():
            t.remove_from_cache()
            uuids.append(t.uuid)
        self._thing_types = {}

        for t in value:
            self._thing_types[t.uuid] = t
            t.parent = self
            if t.uuid in uuids:
                uuids.remove(t.uuid)

        for u in uuids:
            self._orphan_list.add(u,'thing_type')

        self.updated_attribute('thing_types','set')

    @property
    def waypoints(self):
        return self._waypoints.values()

    @waypoints.setter
    def waypoints(self, value):
        uuids = []

        for w in self._waypoints.values():
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
            self._orphan_list.add(u,'trajectory')

        self.updated_attribute('trajectories','set')

    @property
    def regions(self):
        return self._regions.values()

    @regions.setter
    def regions(self, value):
        uuids = []

        for r in self._regions.values():
            r.remove_from_cache()
            uuids.append(r.uuid)

        self._regions = {}

        for r in value:
            self._regions[r.uuid] = r
            r.parent - self
            if r.uuid in uuids:
                uuids.remove(r.uuid)

        for u in uuids:
            self._orphan_list.add(u,'region')

    @property
    def grade_types(self):
        return self._grade_types.values()

    @grade_types.setter
    def grade_types(self, value):
        uuids = []

        for g in self._grade_types.values():
            g.remove_from_cache()
            uuids.append(g.uuid)

        self._grade_types = {}

        for g in value:
            self._grade_types[g.uuid] = g
            g.parent = self
            if g.uuid in uuids:
                uuids.remove(g.uuid)

        for u in uuids:
            self._orphan_list.add(u,'grade_type')

    @property
    def placeholders(self):
        return self._placeholders.values()

    @placeholders.setter
    def placeholders(self, value):
        uuids = []

        for p in self._placeholders.values():
            p.remove_from_cache()
            uuids.append(p.uuid)

        self._placeholders = {}

        for p in value:
            self._placeholders[p.uuid] = p
            p.paren = self
            if p.uuid in uuids:
                uuids.remove(p.uuid)

        for u in uuids:
            self._orphan_list.add(u,'grade_type')

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

    def get_thing_type(self, uuid):
        if uuid in self._thing_types.keys():
            return self._thing_types[uuid]
        else:
            return None

    def add_thing_type(self, thing_type):
        thing_type.parent = self
        self._thing_types[thing_type.uuid] = thing_type
        self.updated_attribute('thing_types','add',thing_type.uuid)

    def delete_thing_type(self, uuid):
        if uuid in self._thing_types.keys():
            self._thing_types.pop(uuid).remove_from_cache()
            self._orphan_list.add(uuid,'thing_type')
            self.updated_attribute('thing_types','delete', uuid)

    def get_region(self, uuid):
        if uuid in self._regions.keys():
            return self._regions[uuid]
        else:
            return None

    def add_region(self, region):
        region.parent = self
        self._regions[region.uuid] = region
        self.updated_attribute('regions','add',region.uuid)

    def delete_region(self, uuid):
        if uuid in self._regions.keys():
            self._regions.pop(uuid).remove_from_cache()
            self._orphan_list.add(uuid,'region')
            self.updated_attribute('regions','delete', uuid)

    def get_grade_type(self, uuid):
        if uuid in self._grade_types.keys():
            return self._grade_types[uuid]
        else:
            return None

    def add_grade_type(self, grade_type):
        grade_type.parent = self
        self._grade_types[grade_type.uuid] = grade_type
        self.updated_attribute('grade_types','add',grade_type.uuid)

    def delete_grade_type(self, uuid):
        if uuid in self._grade_types.keys():
            self._grade_types.pop(uuid).remove_from_cache()
            self._orphan_list.add(uuid,'grade_type')
            self.updated_attribute('grade_types','delete', uuid)

    def get_placeholder(self, uuid):
        if uuid in self._placeholders.keys():
            return self._placeholders[uuid]
        else:
            return None

    def add_placeholder(self, placeholder):
        placeholder.parent = self
        self._placeholders[placeholder.uuid] = placeholder
        self.updated_attribute('placeholders','add',placeholder.uuid)

    def delete_placeholder(self, uuid):
        if uuid in self._placeholders.keys():
            self._placeholders.pop(uuid).remove_from_cache()
            self._orphan_list.add(uuid,'placeholder')
            self.updated_attribute('placeholder','delete',uuid)

    def set(self, dct):
        if 'locations' in dct.keys():
            self.locations = [NodeParser(l, enforce_types=[Location.type_string(trailing_delim=False)]) for l in dct['locations']]

        if 'machines' in dct.keys():
            self.machines = [NodeParser(m, enforce_types=[Machine.type_string(trailing_delim=False)]) for m in dct['machines']]

        if 'things' in dct.keys():
            self.things = [NodeParser(t, enforce_types=[Thing.type_string(trailing_delim=False)]) for t in dct['things']]

        if 'waypoints' in dct.keys():
            self.waypoints = [NodeParser(w, enforce_types=[Waypoint.type_string(trailing_delim=False)]) for w in dct['waypoints']]

        if 'trajectories' in dct.keys():
            self.trajectories = [NodeParser(t, enforce_types=[Trajectory.type_string(trailing_delim=False)]) for t in dct['trajectories']]

        if 'thing_types' in dct.keys():
            self.thing_types = [NodeParser(t, enforce_types=[ThingType.type_string(trailing_delim=False)]) for t in dct['thing_types']]

        if 'regions' in dct.keys():
            self.regions = [NodeParser(r, enforce_types=[ALL_REGION_TYPES]) for r in dct['regions']]

        if 'grade_types' in dct.keys():
            self.grade_types = [NodeParser(g, enforce_types=[GradeType.type_string(trailing_delim=False)]) for g in dct['grade_types']]

        if 'placeholders' in dct.keys():
            self.placeholders = [NodeParser(p, enforce_types=[Placeholder.type_string(trailing_delim=False)]) for p in dct['placeholders']]

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

        for t in self.thing_types:
            t.remove_from_cache()

        for r in self.regions:
            r.remove_from_cache()

        for g in self.grade_types:
            g.remove_from_cache()

        for p in self.placeholders:
            p.remove_from_cache()

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

        for t in self.thing_types:
            t.add_to_cache()

        for r in self.regions:
            r.add_to_cache()

        for g in self.grade_types:
            g.add_to_cache()

        for p in self.placeholders:
            p.add_to_cache()

        super(Context,self).add_to_cache()

    '''
    Children methods
    '''

    def delete_child(self, uuid):
        success = True

        if uuid in [l.uuid for l in self.locations]:
            self.delete_location(uuid)
        elif uuid in [m.uuid for m in self.machines]:
            self.delete_machine(uuid)
        elif uuid in [t.uuid for t in self.things]:
            self.delete_thing(uuid)
        elif uuid in [w.uuid for w in self.waypoints]:
            self.delete_waypoint(uuid)
        elif uuid in [t.uuid for t in self.trajectories]:
            self.delete_trajectory(uuid)
        elif uuid in [t.uuid for t in self.thing_types]:
            self.delete_thing_type(uuid)
        elif uuid in [r.uuid for r in self.regions]:
            self.delete_region(uuid)
        elif uuid in [g.uuid for g in self.grade_types]:
            self.delete_grade_type(uuid)
        elif uuid in [p.uuid for p in self.placeholders]:
            self.delete_placeholder(uuid)
        else:
            success = super(Context,self).delete_child(uuid)

        return success

    def add_child(self, node):
        success = True

        if isinstance(node,Location) and node.uuid not in [l.uuid for l in self.locations]:
            self.add_location(node)
        elif isinstance(node,Machine) and node.uuid not in [m.uuid for m in self.machines]:
            self.add_machine(node)
        elif isinstance(node,Thing) and node.uuid not in [t.uuid for t in self.things]:
            self.add_thing(node)
        elif isinstance(node,Waypoint) and node.uuid not in [w.uuid for w in self.waypoints]:
            self.add_waypoint(node)
        elif isinstance(node,Trajectory) and node.uuid not in [t.uuid for t in self.trajectories]:
            self.add_trajectory(node)
        elif isinstance(node,ThingType) and node.uuid not in [t.uuid for t in self.thing_types]:
            self.add_thing_type(node)
        elif isinstance(node,Region) and node.uuid not in [r.uuid for r in self.regions]:
            self.add_region(node)
        elif isinstance(node,GradeType) and node.uuid not in [g.uuid for g in self.grade_types]:
            self.add_grade_type(node)
        elif isinstance(node,Placeholder) and node.uuid not in [p.uuid for p in self.placeholders]:
            self.add_placeholder(node)
        else:
            success = super(Context,self).add_child(node)

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

        for t in self.thing_types:
            t.late_construct_update()

        for r in self.regions:
            r.late_construct_update()

        for g in self.grade_types:
            g.late_construct_update()

        for p in self.placeholders:
            p.late_construct_update()

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

        for t in self.thing_types:
            t.deep_update()

        for r in self.regions:
            r.deep_update()

        for g in self.grade_types:
            g.deep_update()

        for p in self.placeholders:
            p.deep_update()

        super(Context,self).deep_update()

        self.updated_attribute('machines','update')
        self.updated_attribute('locations','update')
        self.updated_attribute('things','update')
        self.updated_attribute('waypoints','update')
        self.updated_attribute('trajectories','update')
        self.updated_attribute('thing_types','update')
        self.updated_attribute('regions','update')
        self.updated_attribute('grade_types','update')
        self.updated_attribute('placeholders','update')

    def shallow_update(self):
        super(Context,self).shallow_update()

        self.updated_attribute('machines','update')
        self.updated_attribute('locations','update')
        self.updated_attribute('things','update')
        self.updated_attribute('waypoints','update')
        self.updated_attribute('trajectories','update')
        self.updated_attribute('thing_types','update')
        self.updated_attribute('regions','update')
        self.updated_attribute('grade_types','update')
        self.updated_attribute('placeholders','update')
