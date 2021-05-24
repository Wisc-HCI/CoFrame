'''
Provides access to any node within EvD by UUID. This is a flattend structure
that is kept coherent with EvDScript's AST.

All nodes can be accessed without a type hint. Additionally, major types can
be more quickly accessed by supplying their hint. Though if a hint is supplied
and the type is not found there is not a fall back to untyped search. Instead,
the same lookup will need to be requested again but under the un-hinted condition.
'''


# Cache is global so that we can keep a UUID list for NodeParser
cacheObj = None
def get_evd_cache_obj():
    global cacheObj

    if cacheObj == None:
        cacheObj = Cache()

    return cacheObj


from .data.trajectory import Trajectory
from .data.location import Location
from .data.waypoint import Waypoint
from .data.thing import Thing
from .data.trace import Trace
from .data.machine import Machine

from .environment.environment import Environment
from .program.program import Program
from .program.primitive import Primitive
from .context import Context


class Cache(object):

    def __init__(self):
        self.data = {}
        self.trajectories = {}
        self.locations = {}
        self.waypoints = {}
        self.things = {}
        self.traces = {}
        self.machines = {}
        self.environments = {}
        self.programs = {}
        self.primitives = {}
        self.contexts = {}

    def add(self, uuid, node):
        self.data[uuid] = node

        if isinstance(node,Trajectory):
            self.trajectories[uuid] = node

        if isinstance(node,Location):
            self.locations[uuid] = node

        if isinstance(node,Waypoint):
            self.waypoints[uuid] = node

        if isinstance(node,Thing):
            self.things[uuid] = node

        if isinstance(node,Trace):
            self.traces[uuid] = node

        if isinstance(node,Machine):
            self.machines[uuid] = node

        if isinstance(node,Program):
            self.programs[uuid] = node

        if isinstance(node,Environment):
            self.environments[uuid] = node

        if isinstance(node,Primitive):
            self.primitives[uuid] = node

        if isinstance(node,Context):
            self.contexts[uuid] = node

    def remove(self, uuid):

        node = self.data.pop(uuid, None)

        if isinstance(node,Trajectory):
            self.trajectories.pop(uuid,None)

        if isinstance(node,Location):
            self.locations.pop(uuid, None)

        if isinstance(node,Waypoint):
            self.waypoints.pop(uuid, None)

        if isinstance(node,Thing):
            self.things.pop(uuid, None)

        if isinstance(node,Trace):
            self.traces.pop(uuid, None)

        if isinstance(node,Machine):
            self.machines.pop(uuid, None)

        if isinstance(node,Program):
            self.programs.pop(uuid, None)

        if isinstance(node,Environment):
            self.environments.pop(uuid, None)

        if isinstance(node,Primitive):
            self.primitives.pop(uuid, None)

        if isinstance(node,Context):
            self.contexts.pop(uuid, None)

    def clear(self):
        self.data = {}
        self.locations = {}
        self.waypoints = {}
        self.trajectories = {}
        self.things = {}
        self.traces = {}
        self.machines = {}
        self.programs = {}
        self.environments = {}
        self.primitives = {}
        self.contexts = {}

    def get(self, uuid, hint=None):
        retVal = None;

        if hint == 'trajectory' and uuid in self.trajectories.keys():
            retVal = self.trajectories[uuid]
        elif hint == 'location' and uuid in self.locations.keys():
            retVal = self.locations[uuid]
        elif hint == 'waypoint' and uuid in self.waypoints.keys():
            retVal = self.waypoints[uuid]
        elif hint == 'thing' and uuid in self.things.keys():
            retVal = self.things[uuid]
        elif hint == 'trace' and uuid in self.traces.keys():
            retVal = self.traces[uuid]
        elif hint == 'machine' and uuid in self.machines.keys():
            retVal = self.machines[uuid]
        elif hint == 'program' and uuid in self.programs.keys():
            retVal = self.programs[uuid]
        elif hint == 'environment' and uuid in self.environments.keys():
            retVal = self.environments[uuid]
        elif hint == 'primitive' and uuid in self.primitives.keys():
            retVal = self.primitives[uuid]
        elif hint == 'context' and uuid in self.contexts.keys():
            retVal = self.contexts[uuid]
        else:
            retVal = self.data[uuid]

        return retVal

    def set(self, uuid, dct, hint=None):
        self.get(uuid,hint).set(dct)

    def utility_cache_stats(self):
        log = {
            'data': {},
            'num_trajectories': 0,
            'num_locations': 0,
            'num_waypoints': 0,
            'num_things': 0,
            'num_traces': 0,
            'num_machines': 0,
            'num_primitives': 0,
            'num_contexts': 0,
        }

        for n in self.data.values():
            if type(n) not in log['data'].keys():
                log['data'][type(n)] = 0

            log['data'][type(n)] += 1

        log['num_trajectories'] = len(self.trajectories)
        log['num_locations'] = len(self.locations)
        log['num_waypoints'] = len(self.waypoints)
        log['num_things'] = len(self.things)
        log['num_traces'] = len(self.traces)
        log['num_machines'] = len(self.machines)
        log['num_primitives'] = len(self.primitives)
        log['num_contexts'] = len(self.contexts)

        return log
