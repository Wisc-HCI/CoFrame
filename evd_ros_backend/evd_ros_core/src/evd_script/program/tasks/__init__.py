from .close_gripper import CloseGripper
from .initialize import Initialize
from .open_gripper import OpenGripper
from .simple_pick_and_place import SimplePickAndPlace


def TasksNodeParser(exactType, dct):

    node = None

    if exactType == "close-gripper":
        node = CloseGripper.from_dct(dct)
    elif exactType == "open-gripper":
        node = OpenGripper.from_dct(dct)
    elif exactType == "simple-pick-and-place":
        node = SimplePickAndPlace.from_dct(dct)
    elif exactType == "initialize":
        node = Initialize.from_dct(dct)

    return node
