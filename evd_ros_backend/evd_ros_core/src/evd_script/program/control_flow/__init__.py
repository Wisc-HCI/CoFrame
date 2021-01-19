from .branch import Branch
from .breakpoint import Breakpoint
from .loop import Loop

from .conditionals import *


def ControlFlowNodeParser(exactType, dct):

    node = ConditionalNodeParser(exactType, dct)
    if node != None:
        return node

    if exactType == "loop":
        node = Loop.from_dct(dct)
    elif exactType == "branch":
        node = Branch.from_dct(dct)
    elif exactType == "breakpoint":
        node = Breakpoint.from_dct(dct)

    return node
