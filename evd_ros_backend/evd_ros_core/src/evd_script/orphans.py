from .cache import *

from evd_ros_core.msg import Issue

# Orphan List is global so we can keep track of potential program errors
orphanList = None
def evd_orphan_list():
    global orphanList

    if orphanList == None:
        orphanList = OrphanList()

    return orphanList


class OrphanList(object):

    def __init__(self):
        self._orphan_uuids = {}

    def add(self, orphan_uuid, type=None):
        if orphan_uuid in self._orphan_uuids.keys() and self._orphan_uuids[orphan_uuid] != type and type != None:
            self._orphan_uuids[orphan_uuid] = type
        elif orphan_uuid not in self._orphan_uuids.keys():
            self._orphan_uuids[orphan_uuid] = type

    def clear(self, orphan_uuid):
        if orphan_uuid in self._orphan_uuids.keys():
            self._orphan_uuids.pop(orphan_uuid)

    def get(self):
        return self._orphan_uuids

    def empty(self):
        return len(self._orphan_uuids) == 0


# Repairs the current cache with the orphan list - could be a time expensive process
def evd_orphan_repair():
    cache = get_evd_cache_obj()

    issues = []

    for uuid in self._orphan_uuids.keys():
        type = self._orphan_uuids[uuid]

        if type == 'location':
            # See if a reference exists
            referenceExists = False
            for context in cache.contexts.values():
                pass

            # Iterate through all location users
            nodeList = []
            nodeList += cache.trajectories.values()
            nodeList += cache.primitives.values()
            for element in nodeList:
                # If not in any context then purge from users
                # If is in a context, then check if accessible
                if referenceExists:
                    pass
                else:
                    pass

        elif type == 'waypoint':
            pass

        elif type == "machine":
            pass

        elif type == "thing":
            pass

        elif type == "trajectory":
            pass
