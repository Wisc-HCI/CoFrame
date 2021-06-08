from .close_gripper import CloseGripper
from .initialize import Initialize
from .open_gripper import OpenGripper
from .simple_pick_and_place import SimplePickAndPlace


def HierarchicalTasksNodeParser(exactType, dct):

    node = None

    if exactType == CloseGripper.type_string(trailing_delim=False):
        node = CloseGripper.from_dct(dct)
    elif exactType == OpenGripper.type_string(trailing_delim=False):
        node = OpenGripper.from_dct(dct)
    elif exactType == SimplePickAndPlace.type_string(trailing_delim=False):
        node = SimplePickAndPlace.from_dct(dct)
    elif exactType == Initialize.type_string(trailing_delim=False):
        node = Initialize.from_dct(dct)

    return node
