from .collision_mesh import CollisionMesh
from .occupancy_zone import OccupancyZone
from .pinch_point import PinchPoint
from .reach_sphere import ReachSphere
from .environment_node import EnvironmentNode


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
    elif exactType == EnvironmentNode.type_string(trailing_delim=False):
        node = EnvironmentNode.from_dct(dct)

    return node
