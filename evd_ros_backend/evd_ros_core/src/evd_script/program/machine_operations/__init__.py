from .machine_blocking_process import MachineBlockingProcess
from .machine_initialize import MachineInitialize
from .machine_primitive import MachinePrimitive
from .machine_start import MachineStart
from .machine_stop import MachineStop
from .machine_wait import MachineWait


def MachineOperationsNodeParser(exactType, dct):

    node = None

    if exactType == MachinePrimitive.type_string(trailing_delim=False):
        node = MachinePrimitive.from_dct(dct)
    elif exactType == MachineStart.type_string(trailing_delim=False):
        node = MachineStart.from_dct(dct)
    elif exactType == MachineWait.type_string(trailing_delim=False):
        node = MachineWait.from_dct(dct)
    elif exactType == MachineStop.type_string(trailing_delim=False):
        node = MachineStop.from_dct(dct)
    elif exactType == MachineInitialize.type_string(trailing_delim=False):
        node = MachineInitialize.from_dct(dct)
    elif exactType == MachineBlockingProcess.type_string(trailing_delim=False):
        node = MachineBlockingProcess.from_dct(dct)

    return node
