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

    def get(self, uuid):
        return self.get(uuid)

    def set(self, uuid, dct):
        self.get(uuid).set(dct)
