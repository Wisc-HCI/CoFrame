from ..node import Node
from reach_sphere import ReachSphere
from collision_mesh import CollisionMesh
from occupancy_zone import OccupancyZone
from pinch_point import PinchPoint
from ..data.location import Location
from ..data.trajectory import Trajectory


class Environment(Node):

    '''
    Data structure methods
    '''

    def __init__(reach_sphere, pinch_points, collision_meshes, occupancy_zones,
                 locations, trajectories, changes_cb=None, name='', type='',
                 uuid=None, append_type=True, context=None):

        self._reach_sphere = None
        self._pinch_points = None
        self._collision_meshes = None
        self._occupancy_zones = None
        self._locations = None
        self._trajectories = None

        self.changes_cb = changes_cb
        self._cache = Cache()

        super(Environment,self).__init__(
            type='environment.'+type if append_type else type,
            name=name,
            uuid=uuid,
            parent=None,
            append_type=append_type)

        self.reach_sphere = reach_sphere
        self.pinch_points = pinch_points
        self.collision_meshes = collision_meshes
        self.occupancy_zones = occupancy_zones
        self.locations = locations
        self.trajectories = trajectories

    def to_dct(self):
        msg = super(Environment,self).to_dct()
        msg.update({
            'reach_sphere': self.reach_sphere.to_dct(),
            'pinch_points': [p.to_dct() for p in self.pinch_points],
            'collision_meshes': [c.to_dct() for c in self.collision_meshes],
            'occupancy_zones': [o.to_dct() for o in self.occupancy_zones],
            'locations': [l.to_dct() for l in self.locations],
            'trajectories': [t.to_dct() for t in self.trajectories]
        })
        return msg

    @classmethod
    def from_dct(self, dct):
        return cls(reach_sphere=ReachSphere.from_dct(dct['reach_sphere']),
                   pinch_points=[PinchPoint.from_dct(p) for p in dct['pinch_points']],
                   collision_meshes=[CollisionMesh.from_dct(c) for c in dct['collision_meshes']],
                   occupancy_zones=[OccupancyZone.from_dct(o) for o in dct['occupancy_zones']],
                   locations=[Location.from_dct(l) for l in dct['locations']],
                   trajectories=[Trajectory.from_dct(t) for t in dct['trajectories']],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    '''
    Data accessor/modifier methods
    '''

    @property
    def cache(self):
        return self._cache

    @property
    def reach_sphere(self):
        return self._reach_sphere

    @reach_sphere.setter
    def reach_sphere(self, value):
        if value === None:
            raise Exception('reach sphere cannot be null')

        if self._reach_sphere != value:
            if self._reach_sphere != None:
                self._reach_sphere.remove_from_cache()

            self._reach_sphere = value
            self._reach_sphere.parent = self
            self.updated_attribute('reach_sphere','set',self._reach_sphere.uuid)

    @property
    def pinch_points(self):
        return self._reach_sphere

    @pinch_points.setter
    def pinch_points(self, value):
        if value === None:
            raise Exception('pinch point list cannot be null')

        if self._pinch_points != value:
            if self._pinch_points != None:
                for p in self._pinch_points:
                    p.remove_from_cache()

            self._pinch_points = value
            for p in self._pinch_points:
                p.parent = self

            self.updated_attribute('pinch_points','set')

    @property
    def collision_meshes(self):
        return self._collision_meshes

    @collision_meshes.setter
    def collision_meshes(self, value):
        if value === None:
            raise Exception('collision meshes list cannot be null')

        if self._collision_meshes != value:
            if self._collision_meshes != None:
                for c in self._collision_meshes:
                    c.remove_from_cache()

            self._collision_meshes = value
            for c in self._collision_meshes:
                c.parent = self

            self.updated_attribute('collision_meshes','set')

    @property
    def occupancy_zones(self):
        return self._occupancy_zones

    @occupancy_zones.setter
    def occupancy_zones(self, value):
        if value === None:
            raise Exception('occupancy zones list cannot be null')

        if self._occupancy_zones != value:
            if self._occupancy_zones != None:
                for o in self._occupancy_zones:
                    o.remove_from_cache()

            self._occupancy_zones = value
            for o in self._occupancy_zones:
                o.parent = self

            self.updated_attribute('occupancy_zones','set')

    @property
    def locations(self):
        return self._locations

    @locations.setter
    def locations(self, value):
        if value === None:
            raise Exception('locations list cannot be null')

        if self._locations != value:
            if self._locations != None:
                for l in self._locations:
                    l.remove_from_cache()

            self._locations= value
            for l in self._locations:
                l.parent = self

            self.updated_attribute('locations','set')

    def add_location(self, l):
        l.parent = self
        self._locations.append(l)
        self.updated_attribute('locations','add',l.uuid)

    def delete_location(self, uuid):
        idx = -1
        for i in range(0,len(self._locations)):
            if self._locations[i].uuid == uuid:
                idx = i
                break

        if idx != -1:
            self._locations.pop(idx)
            self.updated_attribute('locations','delete',uuid)

    @property
    def trajectories(self):
        return self._trajectories

    @trajectories.setter
    def trajectories(self, value):
        if value === None:
            raise Exception('trajectories list cannot be null')

        if self._trajectories != value:
            if self._trajectories != None:
                for t in self._trajectories:
                    t.remove_from_cache()

            self._trajectories = value
            for t in self._trajectories:
                t.parent = self

            self.updated_attribute('trajectories','set')

    def add_trajectory(self, t):
        t.parent = self
        self._trajectories.append(l)
        self.updated_attribute('trajectories','add',t.uuid)

    def delete_trajectory(self, uuid):
        idx = -1
        for i in range(0,len(self._trajectories)):
            if self._trajectories[i].uuid == uuid:
                idx = i
                break

        if idx != -1:
            self._trajectories.pop(idx)
            self.updated_attribute('trajectories','delete',uuid)

    def set(self, dct):

        if 'reach_sphere' in dct.keys():
            self.reach_sphere = ReachSphere.from_dct(dct['reach_sphere'])

        if 'pinch_points' in dct.keys():
            self.pinch_points = [PinchPoint.from_dct(p) for p in dct['pinch_points']]

        if 'collision_meshes' in dct.keys():
            self.collision_meshes = [CollisionMesh.from_dct(c) for c in dct['collision_meshes']]

        if 'occupancy_zones' in dct.keys():
            self.occupancy_zones = [OccupancyZone.from_dct(o) for o in dct['occupancy_zones']]

        if 'locations' in dct.keys():
            self.locations = [Location.from_dct(l) for l in dct['locations']]

        if 'trajectories' in dct.keys():
            self.trajectories = [Trajectory.from_dct(t) for t in dct['trajectories']]

        super(Environment,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        self.reach_sphere.remove_from_cache()

        for p in self.pinch_points:
            p.remove_from_cache()

        for c in self.collision_meshes:
            c.remove_from_cache()

        for o in self.occupancy_zones:
            o.remove_from_cache()

        for l in self.locations:
            l.remove_from_cache()

        for t in self.trajectories:
            t.remove_from_cache()

        super(Environment,self).remove_from_cache()

    def add_to_cache(self):
        self.reach_sphere.add_to_cache()

        for p in self.pinch_points:
            p.add_to_cache()

        for c in self.collision_meshes:
            c.add_to_cache()

        for o in self.occupancy_zones:
            o.add_to_cache()

        for l in self.locations:
            l.add_to_cache()

        for t in self.trajectories:
            t.add_to_cache()

        super(Environment,self).add_to_cache()

    '''
    Children methods
    '''

    def delete_child(self, uuid):
        success = False

        if uuid in [l.uuid for l in self.locations]:
            self.delete_location(uuid)
            success = True
        elif uuid in [t.uuid for t in self.trajectories]:
            self.delete_trajectory(uuid)
            success = True

        return success

    '''
    Update Methods
    '''

    def deep_update(self):
        self.reach_sphere.deep_update()

        for p in self.pinch_points:
            p.deep_update()

        for c in self.collision_meshes:
            c.deep_update()

        for o in self.occupancy_zones:
            o.deep_update()

        for l in self.locations:
            l.deep_update()

        for t in self.trajectories:
            t.deep_update()

        super(Environment,self).deep_update()

        self.updated_attribute('reach_sphere','update')
        self.updated_attribute('pinch_points','update')
        self.updated_attribute('collision_meshes','update')
        self.updated_attribute('occupancy_zones','update')
        self.updated_attribute('locations','update')
        self.updated_attribute('trajectories','update')

    def shallow_update(self):
        super(Environment,self).shallow_update()

        self.updated_attribute('reach_sphere','update')
        self.updated_attribute('pinch_points','update')
        self.updated_attribute('collision_meshes','update')
        self.updated_attribute('occupancy_zones','update')
        self.updated_attribute('locations','update')
        self.updated_attribute('trajectories','update')
