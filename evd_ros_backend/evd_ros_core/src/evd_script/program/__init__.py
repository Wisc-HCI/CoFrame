from .task import Task
from .cache import Cache
from .context import Context
from .primitive import Primitive
from .program import Program

from .tasks import *
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

    node  = TasksNodeParser(exactType, dct)
    if node != None:
        return node

    if exactType == "program":
        node = Program.from_dct(dct)
    elif exactType == "context":
        node = Context.from_dct(dct)
    elif exactType == "task":
        node = Task.from_dct(dct)
    elif exactType == "primitive":
        node = Primitive.from_dct(dct)

    return node
