
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
                'events': {},
                'event_graphs': {}
            }

        self.locations = {key:Location.from_dct(dct['locations'][key]) for key in dct['locations'].keys()}
        self.waypoints = {key:Waypoint.from_dct(dct['waypoints'][key]) for key in dct['waypoints'].keys()}
        self.trajectories = {key:Trajectory.from_dct(dct['trajectories'][key]) for key in dct['trajectories'].keys()}
        self.traces = {key:Trace.from_dct(dct['traces'][key]) for key in dct['traces'].keys()}
        self.events = {key:None for key in dct['events'].keys()} #TODO
        self.event_graphs = (key:None for key in dct['event_graphs'].keys()) #TODO

    def to_dct(self):
        return {
            'uuid': self._uuid,
            'name': self.name,
            'type': self.type,
            'locations': {key:self._locations[key].to_dct() for key in self._locations.keys()},
            'waypoints': {key:self._waypoints[key].to_dct() for key in self._waypoints.keys()},
            'trajectories': {key:self._trajectories[key].to_dct() for key in self._trajectories.keys()},
            'traces': {key:self._traces[key].to_dct() for key in self._traces.keys()},
            'events': {key:self._events[key].to_dct() for key in self._events.keys()},
            'event_graphs': {key:self._event_graphs[key].to_dct() for key in self._event_graphs.keys()}
        }

    @classmethod
    def from_dct(cls, dct):
        return cls(
            name=dct['name'],
            uuid=dct['uuid'],
            dct)
