from ..data.trajectory import Trajectory
from ..data.location import Location


class Cache(object):

    def __init__(self):
        self.data = {}
        self.trajectories = {}
        self.locations = {}

    def add(self, uuid, node):
        self.data[uuid] = node

        if isinstance(node,Location):
            self.locations[uuid] = node
            #print self.locations
        elif isinstance(node,Trajectory):
            self.trajectories[uuid] = node

    def remove(self, uuid):
        node = self.data.pop(uuid, None)
        if isinstance(node,Location):
            self.locations.pop(uuid, None)
        elif isinstance(node,Trajectory):
            self.trajectories.pop(uuid,None)

    def clear(self):
        self.data = {}
        self.locations = {}
        self.trajectories = {}

    def get(self, uuid, hint=None):

        if hint == 'trajectory' and uuid in self.trajectories.keys():
            return self.trajectories[uuid]
        elif hint == 'location' and uuid in self.locations.keys():
            return self.locations[uuid]
        else:
            return self.data[uuid]

    def set(self, uuid, dct, hint=None):
        self.get(uuid,hint).set(dct)
