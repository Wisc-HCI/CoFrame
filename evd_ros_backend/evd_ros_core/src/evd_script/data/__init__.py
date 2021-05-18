from .geometry import *
from .location import Location
from .waypoint import Waypoint
from .machine import Machine, MachineRecipe
from .trace import Trace
from .trace_data_point import TraceDataPoint
from .trajectory import Trajectory
from .regions import *
from .thing import Thing


def DataNodeParser(exactType, dct):
    node = None

    node = GeometryNodeParser(exactType,dct)
    if node != None:
        return node

    node = RegionsNodeParser(exactType,dct)
    if node != None:
        return node

    if exactType == Waypoint.type_string(trailing_delim=False):
        node = Waypoint.from_dct(dct)
    elif exactType == Location.type_string(trailing_delim=False):
        node = Location.from_dct(dct)
    elif exactType == Trajectory.type_string(trailing_delim=False):
        node = Trajectory.from_dct(dct)
    elif exactType == Trace.type_string(trailing_delim=False):
        node = Trace.from_dct(dct)
    elif exactType == TraceDataPoint.type_string(trailing_delim=False):
        node = TraceDataPoint.from_dct(dct)
    elif exactType == Machine.type_string(trailing_delim=False):
        node = Machine.from_dct(dct)
    elif exactType == MachineRecipe.type_string(trailing_delim=False):
        node  = MachineRecipe.from_dct(dct)
    elif exactType == Thing.type_string(trailing_delim=False):
        node = Thing.from_dct(dct)

    return node
