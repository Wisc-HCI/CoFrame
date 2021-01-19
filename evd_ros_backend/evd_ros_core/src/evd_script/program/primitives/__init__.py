from .delay import Delay
from .gripper import Gripper
from .move_trajectory import MoveTrajectory
from .move_unplanned import MoveUnplanned


def PrimitivesNodeParser(exactType, dct):

    node = None

    if exactType == "move-trajectory":
        node = MoveTrajectory.from_dct(dct)
    elif exactType == "move-unplanned":
        node = MoveUnplanned.from_dct(dct)
    elif exactType == "delay":
        node = Delay.from_dct(dct)
    elif exactType == "gripper":
        node = Gripper.from_dct(dct)

    return node
