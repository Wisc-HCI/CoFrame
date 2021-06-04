from .hierarchical import Hierarchical
from .primitive import Primitive
from .skill import Skill

from .hierarchical_tasks import *
from .primitives import *
from .machine_operations import *
from .control_flow import *
from .predefined_skills import *


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

    node = HierarchicalTasksNodeParser(exactType, dct)
    if node != None:
        return node

    node = PredefinedSkillsNodeParser(exactType, dct)
    if node != None:
        return node

    if exactType == Hierarchical.type_string(trailing_delim=False):
        node = Hierarchical.from_dct(dct)
    elif exactType == Primitive.type_string(trailing_delim=False):
        node = Primitive.from_dct(dct)
    elif exactType == Skill.type_string(trailing_delim=False):
        node = Skill.from_dct(dct)

    return node
