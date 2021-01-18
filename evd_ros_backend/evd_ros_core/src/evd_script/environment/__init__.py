from .environment import Environment
from .collision_mesh import CollisionMesh
from .occupancy_zone import OccupancyZone
from .pinch_point import PinchPoint
from .reach_sphere import ReachSphere


def EnvironmentNodeParser(exactType, dct):

    node = None

    if exactType == "reach-sphere":
        node = ReachSphere.from_dct(dct)
    elif exactType == "pinch-point":
        node = PinchPoint.from_dct(dct)
    elif exactType == "collision-mesh":
        node = CollisionMesh.from_dct(dct)
    elif exactType == "occupancy-zone":
        node = OccupancyZone.from_dct(dct)
    elif exactType == "environment":
        node = Environment.from_dct(dct)

    return node
