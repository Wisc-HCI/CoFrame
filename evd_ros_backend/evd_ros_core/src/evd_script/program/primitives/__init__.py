from .delay import Delay
from .gripper import Gripper
from .move_trajectory import MoveTrajectory
from .move_unplanned import MoveUnplanned


def PrimitivesNodeParser(exactType, dct):

    node = None

    if exactType == MoveTrajectory.type_string(trailing_delim=False):
        node = MoveTrajectory.from_dct(dct)
    elif exactType == MoveUnplanned.type_string(trailing_delim=False):
        node = MoveUnplanned.from_dct(dct)
    elif exactType == Delay.type_string(trailing_delim=False):
        node = Delay.from_dct(dct)
    elif exactType == Gripper.type_string(trailing_delim=False):
        node = Gripper.from_dct(dct)

    return node
