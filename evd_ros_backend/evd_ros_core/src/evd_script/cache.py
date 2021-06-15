'''
Provides access to any node within EvD by UUID. This is a flattend structure
that is kept coherent with EvDScript's AST.

All nodes can be accessed without a type hint. Additionally, major types can
be more quickly accessed by supplying their hint.
'''


# Cache is global so that we can keep a UUID list for NodeParser
cacheObj = None
def get_evd_cache_obj():
    global cacheObj

    if cacheObj == None:
        cacheObj = Cache()

    return cacheObj


from .data_nodes.trajectory import Trajectory
from .data_nodes.location import Location
from .data_nodes.waypoint import Waypoint
from .data_nodes.thing import Thing
from .data_nodes.trace import Trace
from .data_nodes.machine import Machine
from .data_nodes.thing_type import ThingType
from .data_nodes.regions.region import Region
from .program_nodes.skill import Skill
from .program_nodes.primitive import Primitive
from .environment_nodes.environment_node import EnvironmentNode
from .program import Program
from .context import Context
from .environment import Environment



class Cache(object):

    def __init__(self):
        self.data = {}
        self.instance_table = {
            'trajectory': {'class': Trajectory, 'table': {}},
            'location': {'class': Location, 'table': {}},
            'waypoint': {'class': Waypoint, 'table': {}},
            'thing': {'class': Thing, 'table': {}},
            'trace': {'class': Trace, 'table': {}},
            'machine': {'class': Machine, 'table': {}},
            'environment': {'class': Environment, 'table': {}},
            'program': {'class': Program, 'table': {}},
            'primitive': {'class': Primitive, 'table': {}},
            'context': {'class': Context, 'table': {}},
            'thing_type': {'class': ThingType, 'table': {}},
            'environment_node': {'class': EnvironmentNode, 'table': {}},
            'skill': {'class': Skill, 'table': {}},
            'region': {'class': Region, 'table': {}}
        }

    def add(self, uuid, node):
        self.data[uuid] = node

        for entry in self.instance_table.values():
            if isinstance(node,entry['class']):
                entry['table'][uuid] = node

    def remove(self, uuid):

        node = self.data.pop(uuid, None)

        for entry in self.instance_table.values():
            if isinstance(node,entry['class']):
                entry['table'].pop(uuid, None)

    def clear(self):
        self.data = {}

        for entry in self.instance_table.values():
            entry['table'] = {}

    def get(self, uuid, hint=None):
        retVal = None

        if hint in self.instance_table.keys() and uuid in self.instance_table[hint]['table'].keys():
            retVal = self.instance_table['hint']['table'][uuid]
        else:
            retVal = self.data[uuid]

        return retVal

    def set(self, uuid, dct, hint=None):
        self.get(uuid,hint).set(dct)

    def get_uuids(self, hint=None):
        retVal = None

        if hint in self.instance_table.keys():
            retVal = self.instance_table[hint]['table'].keys()
        elif hint == None:
            retVal = self.data.keys()

        return retVal

    def utility_cache_stats(self):
        log = {'data': {}}

        for n in self.data.values():
            if type(n) not in log['data'].keys():
                log['data'][type(n)] = 0

            log['data'][type(n)] += 1

        for key, entry in self.instance_table.items():
            log['num_' + key] = len(entry['table'])

        return log
