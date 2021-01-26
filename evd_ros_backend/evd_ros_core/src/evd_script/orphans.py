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


# Repairs the current program and cache with the orphan list - could be a time expensive process
def evd_orphan_repair():
    from .program.primitives.gripper import Gripper
    from .program.primitives.move_trajectory import MoveTrajectory
    from .program.machine_operations.machine_primitive import MachinePrimitive

    cache = get_evd_cache_obj()

    for uuid in self._orphan_uuids.keys():
        type = self._orphan_uuids[uuid]

        if type == 'location':
            found = False
            for env in cache.environments.values():
                for loc in env.locations:
                    if loc.uuid == uuid:
                        found = True
                        break

            if not found:
                for traj in cache.trajectories.values():
                    if traj.start_location_uuid == uuid:
                        traj.start_location_uuid = None

                    if traj.end_location_uuid == uuid:
                        traj.end_location_uuid = None

                for prim in cache.primitives.values():
                    if isinstance(prim,MoveTrajectory):
                        if prim.start_location_uuid == uuid:
                            prim.start_location_uuid = None

                        if prim.end_location_uuid == uuid:
                            prim.end_location_uuid = None

        elif type == 'waypoint':
            found = False
            for env in cache.environments.values():
                for way in env.waypoints:
                    if way.uuid == uuid:
                        found = True
                        break

            if not found:
                for traj in cache.trajectories.values():
                    traj.delete_waypoint_uuid(uuid) # will do nothing if not there

        elif type == "machine":
            found = False
            for env in cache.environments.values():
                for mach in env.machines:
                    if mach.uuid == uuid:
                        found = True
                        break

            if not found:
                for prim in cache.primitives.values():

                    if isinstance(prim,MachinePrimitive):
                        if prim.machine_uuid == uuid:
                            prim.machine_uuid = None

        elif type == "thing":

            found = False
            for env in cache.environments.values():
                for thing in env.things:
                    if thing.uuid == uuid:
                        found = True
                        break

            if not found:
                for prim in cache.primitives.values():

                    if isinstance(prim,Gripper):
                        if prim.thing_uuid == uuid:
                            prim.thing_uuid = None

        elif type == "trajectory":

            shouldDelete = True
            for prim in cache.primitives.values():
                 if isinstance(prim,MoveTrajectory):
                     if prim.trajectory_uuid == uuid:
                         shouldNotDelete = False
                         break

            if shouldDelete:
                for env in cache.environments.values():
                    env.delete_trajectory(uuid)
