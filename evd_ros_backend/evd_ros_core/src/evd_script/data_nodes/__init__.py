from .geometry import *
from .regions import *

from .location import Location
from .waypoint import Waypoint
from .machine import Machine
from .trace import Trace
from .trajectory import Trajectory
from .thing import Thing
from .thing_type import ThingType
from .grade_type import GradeType
from .skill_argument import SkillArgument


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
    elif exactType == Machine.type_string(trailing_delim=False):
        node = Machine.from_dct(dct)
    elif exactType == Thing.type_string(trailing_delim=False):
        node = Thing.from_dct(dct)
    elif exactType == ThingType.type_string(trailing_delim=False):
        node = ThingType.from_dct(dct)
    elif exactType == GradeType.type_string(trailing_delim=False):
        node = GradeType.from_dct(dct)
    elif exactType == SkillArgument.type_string(trailing_delim=False):
        node = SkillArgument.from_dct(dct)

    return node
