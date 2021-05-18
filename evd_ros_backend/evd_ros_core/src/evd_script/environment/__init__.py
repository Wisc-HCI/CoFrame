from .environment import Environment
from .collision_mesh import CollisionMesh
from .occupancy_zone import OccupancyZone
from .pinch_point import PinchPoint
from .reach_sphere import ReachSphere


def EnvironmentNodeParser(exactType, dct):

    node = None

    if exactType == ReachSphere.type_string(trailing_delim=False):
        node = ReachSphere.from_dct(dct)
    elif exactType == PinchPoint.type_string(trailing_delim=False):
        node = PinchPoint.from_dct(dct)
    elif exactType == CollisionMesh.type_string(trailing_delim=False):
        node = CollisionMesh.from_dct(dct)
    elif exactType == OccupancyZone.type_string(trailing_delim=False):
        node = OccupancyZone.from_dct(dct)
    elif exactType == Environment.type_string(trailing_delim=False):
        node = Environment.from_dct(dct)

    return node
