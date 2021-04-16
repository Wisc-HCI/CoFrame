from .skill import Skill
from .primitive import Primitive
from .program import Program

from .skills import *
from .primitives import *
from .machine_operations import *
from .control_flow import *


def ProgramNodeParser(exactType, dct):

    node = ControlFlowNodeParser(exactType, dct)
    if node != None:
        return node

    node = MachineOperationsNodeParser(exactType, dct)
    if node != None:
        return node

    node = PrimitivesNodeParser(exactType, dct)
    if node != None:
        return node

    node  = SkillsNodeParser(exactType, dct)
    if node != None:
        return node

    if exactType == "program":
        node = Program.from_dct(dct)
    elif exactType == "skill":
        node = Skill.from_dct(dct)
    elif exactType == "primitive":
        node = Primitive.from_dct(dct)

    return node
