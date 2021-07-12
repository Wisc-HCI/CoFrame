from .skill import Skill
from .primitive import Primitive
from .hierarchical import Hierarchical
from .machine_primitive import MachinePrimitive

from .primitives import *
from .control_flow import *
from .skills import *


primitive_library = [
    Delay,
    Gripper,
    MoveTrajectory,
    MoveUnplanned,
    Hierarchical,
    Loop,
    SkillCall,
    Breakpoint,
    MachineInitialize,
    MachineStart,
    MachineWait,
    MachineStop
]


skills_library = [
    MachineBlockingProcess,
    SimplePickAndPlace,
    OpenGripper,
    CloseGripper,
    Initialize
]


def ProgramNodeParser(exactType, dct):

    node = ControlFlowNodeParser(exactType, dct)
    if node != None:
        return node

    node = PrimitivesNodeParser(exactType, dct)
    if node != None:
        return node

    if exactType == Hierarchical.type_string(trailing_delim=False):
        node = Hierarchical.from_dct(dct)
    elif exactType == Primitive.type_string(trailing_delim=False):
        node = Primitive.from_dct(dct)
    elif exactType == Skill.type_string(trailing_delim=False):
        node = Skill.from_dct(dct)
    elif exactType == MachinePrimitive.type_string(trailing_delim=False):
        node = MachinePrimitive.from_dct(dct)

    return node
