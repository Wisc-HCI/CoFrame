from .data.trajectory import Trajectory
from .data.location import Location
from .data.waypoint import Waypoint
from .data.thing import Thing
from .data.trace import Trace
from .data.machine import Machine


class Cache(object):

    def __init__(self):
        self.data = {}
        self.trajectories = {}
        self.locations = {}
        self.waypoints = {}
        self.things = {}
        self.traces = {}
        self.machines = {}

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

    def clear(self):
        self.data = {}
        self.locations = {}
        self.trajectories = {}
        self.things = {}
        self.traces = {}
        self.machines = {}

    def get(self, uuid, hint=None):

        if hint == 'trajectory' and uuid in self.trajectories.keys():
            return self.trajectories[uuid]
        elif hint == 'location' and uuid in self.locations.keys():
            return self.locations[uuid]
        elif hint == 'waypoint' and uuid in self.waypoints.keys():
            return self.waypoints[uuid]
        elif hint == 'thing' and uuid in self.things.keys():
            return self.things[uuid]
        elif hint == 'traces' and uuid in self.traces.keys():
            return self.traces[uuid]
        elif hint == 'machines' and uuid in self.machines.keys():
            return self.machines[uuid]
        else:
            return self.data[uuid]

    def set(self, uuid, dct, hint=None):
        self.get(uuid,hint).set(dct)
