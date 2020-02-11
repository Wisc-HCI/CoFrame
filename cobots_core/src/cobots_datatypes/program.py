'''

'''
# TODO make sure to update structures on set, delete, create

from task import *
from trace import *
from abstract import *
from geometry import *
from location import *
from waypoint import *
from primitive import *
from trajectory import *


class Program(Abstract):

    def __init__(self, name='', uuid=None, dct=None):
        Abstract.__init__(self,'program',name,uuid)

        if dct == None:
            dct = {
                'locations': {},
                'waypoints': {},
                'trajectories': {},
                'traces': {},
                'primitives': {},
                'root_graph': None
            }

        self.locations = {key:Location.from_dct(dct['locations'][key]) for key in dct['locations'].keys()}
        self.waypoints = {key:Waypoint.from_dct(dct['waypoints'][key]) for key in dct['waypoints'].keys()}
        self.trajectories = {key:Trajectory.from_dct(dct['trajectories'][key]) for key in dct['trajectories'].keys()}
        self.traces = {key:Trace.from_dct(dct['traces'][key]) for key in dct['traces'].keys()}
        self.primitives = {key:PrimitiveFactory(dct['primitives'][key]) for key in dct['primitives'].keys()} #TODO
        self.root_graph = dct['root_graph']

        if self.root_graph == None:
            self.root_graph = Graph()
            self.primitives[self.root_graph.uuid] = self.root_graph

    def __getattribute__(self, field):
        if field == 'locations':
            return self.locations
        elif field == 'waypoints':
            return self.waypoints
        elif field == 'trajectories':
            return self.trajectories
        elif field == 'traces':
            return self.traces
        elif field == 'primitive':
            return self.primitives
        elif field == 'root_graph':
            return self.root_graph
        else:
            raise Exception('Invalid key: {}'.format(field))

    def get(self, field, id):
        if field != 'root_graph':
            return self[field][id]
        else:
            return self[field]

    def set(self, field, id, data):
        if field == 'locations':
            self.locations[id].update(data)
        elif field == 'waypoints':
            self.waypoints[id].update(data)
        elif field == 'trajectories':
            self.trajectories[id].update(data)
        elif field == 'traces':
            self.traces[id].update(data)
        elif field == 'primitives':
            self.primitives[id].update(data)
        elif field == 'root_graph':
            self.root_graph = data

    def create(self, field, data):
        if field == 'locations':
            self.locations[data.uuid] = Location.from_dct(data)
        elif field == 'waypoints':
            self.waypoints[data.uuid] = Waypoint.from_dct(data)
        elif field == 'trajectories':
            self.trajectories[data.uuid] = Trajectory.from_dct(data)
        elif field == 'traces':
            self.traces[data.uuid] = TraceTrajectory.from_dct(data)
        elif field == 'primitives':
            self.primitives[data.uuid] = PrimitiveFactory(data)

    def delete(self, field, id, suppress_upward_chain=False):
        if field == 'locations':
            self.locations.pop(id)
            if not suppress_upward_chain:
                for prm_id in self.primitives.keys():
                    if isinstance(self.primitives[prm_id],MovePrimitive):
                        found = False
                        if id == self.primitives[prm_id].start_location_uuid:
                            self.primitives[prm_id].start_location_uuid = None
                            found = True
                        if id == self.primitives[prm_id].end_location_uuid:
                            self.primitives[prm_id].end_location_uuid = None
                            found = True
                        if found:
                             self.primitives[prm_id].runnable_trajectory_uuid = None
                            for tr_id in self.primitives[prm_id].trajectory_uuids:
                                self.delete('trajectories',tr_id,True)

        elif field == 'waypoints':
            self.waypoints.pop(id)
            if not suppress_upward_chain
                for tr_id in self.trajectories.keys():
                    if id in self.trajectories[tr_id].waypoint_uuids:
                        self.trajectories[tr_id].waypoint_uuids.remove()
                        self.delete('traces',self.trajectories[tr_id].trace_uuid,True)
                        self.trajectories[tr_id].trace_uuid = None

        elif field == 'trajectories':
            for wp_id in self.trajectories[id].waypoint_uuids:
                self.delete('waypoints',wp_id,True)
            self.delete('traces',self.traces[id].trace_uuid,True)
            self.trajectories.pop(id)
            if not suppress_upward_chain:
                for prm_id in self.primitive.keys():
                    if isinstance(self.primitives[prm_id], MovePrimitive):
                        if id in self.primitives[prm_id].trajectory_uuids:
                            self.primitives[prm_id].trajectory_uuids.remove(id)
                        if id == self.primitives[prm_id].runnable_trajectory_uuid:
                            self.primitives[prm_id].runnable_trajectory_uuid = None

        elif field == 'traces':
            self.traces.pop(id)
            if not suppress_upward_chain:
                for tr_id in self.trajectories.keys():
                    if self.trajectories[tr_id].trace_uuid == id:
                        self.trajectories[tr_id].trace_uuid = None

        elif field == 'primitives':
            if isinstance(self.primitives[id],MovePrimitive):
                for tr_id in self.primitives[id].trajectory_uuids:
                    self.delete('trajectories',tr_id,True)
            self.primitives.pop(id)

    def to_dct(self):
        return {
            'uuid': self._uuid,
            'name': self.name,
            'type': self.type,
            'locations': {key:self.locations[key].to_dct() for key in self.locations.keys()},
            'waypoints': {key:self.waypoints[key].to_dct() for key in self.waypoints.keys()},
            'trajectories': {key:self.trajectories[key].to_dct() for key in self.trajectories.keys()},
            'traces': {key:self.traces[key].to_dct() for key in self.traces.keys()},
            'primitives': {key:self.primitives[key].to_dct() for key in self.primitives.keys()},
            'root_graph': self.root_graph
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            dct)
