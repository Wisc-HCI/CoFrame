from .branch import Branch
from .breakpoint import Breakpoint
from .loop import Loop

from .conditionals import *


def ControlFlowNodeParser(exactType, dct):

    node = ConditionalNodeParser(exactType, dct)
    if node != None:
        return node

    if exactType == Loop.type_string(trailing_delim=False):
        node = Loop.from_dct(dct)
    elif exactType == Branch.type_string(trailing_delim=False):
        node = Branch.from_dct(dct)
    elif exactType == Breakpoint.type_string(trailing_delim=False):
        node = Breakpoint.from_dct(dct)

    return node
