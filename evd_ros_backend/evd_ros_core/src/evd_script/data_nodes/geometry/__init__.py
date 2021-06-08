from .pose import Pose
from .joints import Joints
from .position import Position
from .orientation import Orientation


def GeometryNodeParser(exactType, dct):
    node = None

    if exactType == Pose.type_string(trailing_delim=False):
        node = Pose.from_dct(dct)
    elif exactType == Joints.type_string(trailing_delim=False):
        node = Joints.from_dct(dct)
    elif exactType == Position.type_string(trailing_delim=False):
        node = Position.from_dct(dct)
    elif exactType == Orientation.type_string(trailing_delim=False):
        node = Orientation.from_dct(dct)

    return node