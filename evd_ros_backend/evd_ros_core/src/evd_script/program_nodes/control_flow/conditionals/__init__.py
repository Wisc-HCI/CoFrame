from .abstract import AbstractConditional
from .comparison import ComparisonConditional
from .logical import LogicConditional
from .machine import MachineConditional


def ConditionalNodeParser(exactType, dct):
    node = None

    if exactType == 'abstract-conditional':
        node = AbstractConditional.from_dct(dct)
    elif exactType == 'comparison-conditional':
        node = ComparisonConditional.from_dct(dct)
    elif exactType == 'logic-conditional':
        node = LogicConditional.from_dct(dct)
    elif exactType == 'machine-conditional':
        node = MachineConditional.from_dct(dct)

    return node
