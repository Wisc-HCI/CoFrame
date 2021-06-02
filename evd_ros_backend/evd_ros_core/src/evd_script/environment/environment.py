'''
Environment extends context to provide additional global lookup of environment
types. This node is directly used by program to expose top-level state.
'''

from ..context import Context
from .reach_sphere import ReachSphere
from .collision_mesh import CollisionMesh
from .occupancy_zone import OccupancyZone
from .pinch_point import PinchPoint
from ..data.location import Location
from ..data.machine import Machine
from ..data.thing import Thing
from ..data.waypoint import Waypoint
from ..data.trajectory import Trajectory
from ..node_parser import NodeParser


class Environment(Context):

    '''
    Data structure methods
    '''

    @classmethod
    def type_string(cls, trailing_delim=True):
        return 'environment' + ('.' if trailing_delim else '')

    @classmethod
    def full_type_string(cls):
        return Context.full_type_string() + cls.type_string()

    def __init__(self, reach_sphere=None, pinch_points=[], collision_meshes=[], occupancy_zones=[],
                 locations=[], machines=[], things=[], waypoints=[], trajectories=[],
                 name='', type='', uuid=None, parent=None, append_type=True, editable=True, deleteable=True):

        self._reach_sphere = None
        self._pinch_points = None
        self._collision_meshes = None
        self._occupancy_zones = None

        super(Environment,self).__init__(
            locations=locations,
            machines=machines,
            things=things,
            waypoints=waypoints,
            trajectories=trajectories,
            type=Environment.type_string() + type if append_type else type,
            name=name,
            uuid=uuid,
            parent=parent,
            append_type=append_type,
            editable=editable,
            deleteable=deleteable)

        self.reach_sphere = reach_sphere
        self.pinch_points = pinch_points
        self.collision_meshes = collision_meshes
        self.occupancy_zones = occupancy_zones

    def to_dct(self):
        msg = super(Environment,self).to_dct()
        msg.update({
            'reach_sphere': self.reach_sphere.to_dct() if self.reach_sphere != None else None,
            'pinch_points': [p.to_dct() for p in self.pinch_points],
            'collision_meshes': [c.to_dct() for c in self.collision_meshes],
            'occupancy_zones': [o.to_dct() for o in self.occupancy_zones]
        })
        return msg

    @classmethod
    def from_dct(cls, dct):
        return cls(reach_sphere=NodeParser(dct['reach_sphere'], enforce_type=ReachSphere.type_string(trailing_delim=False)) if dct['reach_sphere'] != None else None,
                   pinch_points=[NodeParser(p, enforce_type=PinchPoint.type_string(trailing_delim=False)) for p in dct['pinch_points']],
                   collision_meshes=[NodeParser(c, enforce_type=CollisionMesh.type_string(trailing_delim=False)) for c in dct['collision_meshes']],
                   occupancy_zones=[NodeParser(o, enforce_type=OccupancyZone.type_string(trailing_delim=False)) for o in dct['occupancy_zones']],
                   locations=[NodeParser(l, enforce_type=Location.type_string(trailing_delim=False)) for l in dct['locations']],
                   machines=[NodeParser(m, enforce_type=Machine.type_string(trailing_delim=False)) for m in dct['machines']],
                   things=[NodeParser(t, enforce_type=Thing.type_string(trailing_delim=False)) for t in dct['things']],
                   waypoints=[NodeParser(w, enforce_type=Waypoint.type_string(trailing_delim=False)) for w in dct['waypoints']],
                   trajectories=[NodeParser(t, enforce_type=Trajectory.type_string(trailing_delim=False)) for t in dct['trajectories']],
                   type=dct['type'] if 'type' in dct.keys() else '',
                   append_type=not 'type' in dct.keys(),
                   uuid=dct['uuid'] if 'uuid' in dct.keys() else None,
                   name=dct['name'] if 'name' in dct.keys() else '')

    '''
    Data accessor/modifier methods
    '''

    @property
    def reach_sphere(self):
        return self._reach_sphere

    @reach_sphere.setter
    def reach_sphere(self, value):

        if self._reach_sphere != value:
            if self._reach_sphere != None:
                self._reach_sphere.remove_from_cache()

            self._reach_sphere = value
            if self._reach_sphere != None:
                self._reach_sphere.parent = self

            self.updated_attribute('reach_sphere','set',self._reach_sphere.uuid)

    @property
    def pinch_points(self):
        return self._pinch_points

    @pinch_points.setter
    def pinch_points(self, value):
        if value == None:
            raise Exception('pinch point list cannot be none')

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
        if value == None:
            raise Exception('collision meshes list cannot be none')

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
        if value == None:
            raise Exception('occupancy zones list cannot be none')

        if self._occupancy_zones != value:
            if self._occupancy_zones != None:
                for o in self._occupancy_zones:
                    o.remove_from_cache()

            self._occupancy_zones = value
            for o in self._occupancy_zones:
                o.parent = self

            self.updated_attribute('occupancy_zones','set')

    def set(self, dct):

        if 'reach_sphere' in dct.keys():
            self.reach_sphere = NodeParser(dct['reach_sphere'], enforce_type=ReachSphere.type_string(trailing_delim=False)) if dct['reach_sphere'] != None else None

        if 'pinch_points' in dct.keys():
            self.pinch_points = [NodeParser(p, enforce_type=PinchPoint.type_string(trailing_delim=False)) for p in dct['pinch_points']]

        if 'collision_meshes' in dct.keys():
            self.collision_meshes = [NodeParser(c, enforce_type=CollisionMesh.type_string(trailing_delim=False)) for c in dct['collision_meshes']]

        if 'occupancy_zones' in dct.keys():
            self.occupancy_zones = [NodeParser(o, enforce_type=OccupancyZone.type_string(trailing_delim=False)) for o in dct['occupancy_zones']]

        super(Environment,self).set(dct)

    '''
    Cache methods
    '''

    def remove_from_cache(self):
        if self.reach_sphere != None:
            self.reach_sphere.remove_from_cache()

        for p in self.pinch_points:
            p.remove_from_cache()

        for c in self.collision_meshes:
            c.remove_from_cache()

        for o in self.occupancy_zones:
            o.remove_from_cache()

        super(Environment,self).remove_from_cache()

    def add_to_cache(self):
        if self.reach_sphere != None:
            self.reach_sphere.add_to_cache()

        for p in self.pinch_points:
            p.add_to_cache()

        for c in self.collision_meshes:
            c.add_to_cache()

        for o in self.occupancy_zones:
            o.add_to_cache()

        super(Environment,self).add_to_cache()

    '''
    Update Methods
    '''

    def late_construct_update(self):
        if self.reach_sphere != None:
            self.reach_sphere.late_construct_update()

        for p in self.pinch_points:
            p.late_construct_update()

        for c in self.collision_meshes:
            c.late_construct_update()

        for o in self.occupancy_zones:
            o.late_construct_update()

        super(Environment,self).late_construct_update()

    def deep_update(self):
        if self.reach_sphere != None:
            self.reach_sphere.deep_update()

        for p in self.pinch_points:
            p.deep_update()

        for c in self.collision_meshes:
            c.deep_update()

        for o in self.occupancy_zones:
            o.deep_update()

        super(Environment,self).deep_update()

        self.updated_attribute('reach_sphere','update')
        self.updated_attribute('pinch_points','update')
        self.updated_attribute('collision_meshes','update')
        self.updated_attribute('occupancy_zones','update')

    def shallow_update(self):
        super(Environment,self).shallow_update()

        self.updated_attribute('reach_sphere','update')
        self.updated_attribute('pinch_points','update')
        self.updated_attribute('collision_meshes','update')
        self.updated_attribute('occupancy_zones','update')
