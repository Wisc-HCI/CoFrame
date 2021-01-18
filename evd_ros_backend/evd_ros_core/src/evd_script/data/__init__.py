from .geometry import Pose, Position, Orientation
from .location import Location
from .waypoint import Waypoint
from .machine import Machine, MachineRecipe
from .trace import Trace, TraceDataPoint
from .trajectory import Trajectory
from .region import Region, CubeRegion, SphereRegion
from .thing import Thing


def DataNodeParser(exactType, dct):
    node = None

    if exactType == "waypoint":
        node = Waypoint.from_dct(dct)
    elif exactType == "location":
        node = Location.from_dct(dct)
    elif exactType == "trajectory":
        node = Trajectory.from_dct(dct)
    elif exactType == "trace":
        node = Trace.from_dct(dct)
    elif exactType == "trace-data-point":
        node = TraceDataPoint.from_dct(dct)

    elif exactType == "pose":
        node = Pose.from_dct(dct)
    elif exactType == "position":
        node = Position.from_dct(dct)
    elif exactType == "orientation":
        node = Orientation.from_dct(dct)

    elif exactType == "machine":
        node = Machine.from_dct(dct)
    elif exactType == "machine-recipe":
        node  = MachineRecipe.from_dct(dct)

    elif exactType == "thing":
        node = Thing.from_dct(dct)

    elif exactType == "region":
        node = Region.from_dct(dct)
    elif exactType == "cube-region":
        node = CubeRegion.from_dct(dct)
    elif exactType == "sphere-region":
        node = SphereRegion.from_dct(dct)

    return node
