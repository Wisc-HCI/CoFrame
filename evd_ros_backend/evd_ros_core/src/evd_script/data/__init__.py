from .geometry import Pose, Position, Orientation
from .location import Location
from .waypoint import Waypoint
from .machine import Machine, MachineRecipe
from .trace import Trace
from .trace_data_point import TraceDataPoint
from .trajectory import Trajectory
from .region import Region
from .cube_region import CubeRegion
from .sphere_region import SphereRegion
from .thing import Thing


def DataNodeParser(exactType, dct):
    node = None

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

    elif exactType == Pose.type_string(trailing_delim=False):
        node = Pose.from_dct(dct)
    elif exactType == Position.type_string(trailing_delim=False):
        node = Position.from_dct(dct)
    elif exactType == Orientation.type_string(trailing_delim=False):
        node = Orientation.from_dct(dct)

    elif exactType == Machine.type_string(trailing_delim=False):
        node = Machine.from_dct(dct)
    elif exactType == MachineRecipe.type_string(trailing_delim=False):
        node  = MachineRecipe.from_dct(dct)

    elif exactType == Thing.type_string(trailing_delim=False):
        node = Thing.from_dct(dct)

    elif exactType == Region.type_string(trailing_delim=False):
        node = Region.from_dct(dct)
    elif exactType == CubeRegion.type_string(trailing_delim=False):
        node = CubeRegion.from_dct(dct)
    elif exactType == SphereRegion.type_string(trailing_delim=False):
        node = SphereRegion.from_dct(dct)

    return node
