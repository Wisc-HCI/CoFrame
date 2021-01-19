from .machine_blocking_process import MachineBlockingProcess
from .machine_initialize import MachineInitialize
from .machine_primitive import MachinePrimitive
from .machine_start import MachineStart
from .machine_stop import MachineStop
from .machine_wait import MachineWait


def MachineOperationsNodeParser(exactType, dct):

    node = None

    if exactType == "machine-primitive":
        node = MachinePrimitive.from_dct(dct)
    elif exactType == "machine-start":
        node = MachineStart.from_dct(dct)
    elif exactType == "machine-wait":
        node = MachineWait.from_dct(dct)
    elif exactType == "machine-stop":
        node = MachineStop.from_dct(dct)
    elif exactType == "machine-initialize":
        node = MachineInitialize.from_dct(dct)
    elif exactType =="machine-blocking-process":
        node = MachineBlockingProcess.from_dct(dct)

    return node
