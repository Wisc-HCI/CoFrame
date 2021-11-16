from .delay import Delay
from .gripper import Gripper
from .move_trajectory import MoveTrajectory
from .move_unplanned import MoveUnplanned

from .machine_initialize import MachineInitialize
from .machine_start import MachineStart
from .machine_stop import MachineStop
from .machine_wait import MachineWait


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
    elif exactType == MachineStart.type_string(trailing_delim=False):
        node = MachineStart.from_dct(dct)
    elif exactType == MachineWait.type_string(trailing_delim=False):
        node = MachineWait.from_dct(dct)
    elif exactType == MachineStop.type_string(trailing_delim=False):
        node = MachineStop.from_dct(dct)
    elif exactType == MachineInitialize.type_string(trailing_delim=False):
        node = MachineInitialize.from_dct(dct)

    return node
