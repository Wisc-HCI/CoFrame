from task import *
from primitive import *


def PrimitiveParser(dct):
    exactType = dct['type'].split('.')[0]

    # Search through tasks
    if exactType == 'task':
        return Task.from_dct(dct)
    elif exactType == 'close-gripper':
        return CloseGripper.from_dct(dct)
    elif exactType == 'open-gripper':
        return OpenGripper.from_dct(dct)
    elif exactType == 'pick-and-place':
        return PickAndPlace.from_dct(dct)
    elif exactType == 'initialize':
        return Initialize.from_dct(dct)
    elif exactType == 'machine-blocking-process':
        return MachineBlockingProcess.from_dct(dct)
    elif exactType == 'loop':
        return Loop.from_dct(dct)

    # Search through primitives
    elif exactType == 'primitive':
        return Primitive.from_dct(dct)
    elif exactType == 'move-trajectory':
        return MoveTrajectory.from_dct(dct)
    elif exactType == 'move-unplanned':
        return MoveUnplanned.from_dct(dct)
    elif exactType == 'delay':
        return Delay.from_dct(dct)
    elif exactType == 'gripper':
        return Gripper.from_dct(dct)
    elif exactType == 'machine':
        return Machine.from_dct(dct)
    elif exactType == 'machine-start':
        return MachineStart.from_dct(dct)
    elif exactType == 'machine-wait':
        return MachineWait.from_dct(dct)
    elif exactType == 'machine-stop':
        return MachineStop.from_dct(dct)
    elif exactType == 'machine-initialize':
        return MachineInitialize.from_dct(dct)
    elif exactType == 'breakpoint':
        return Breakpoint.from_dct(dct)

    # Could not resolve
    else:
        return None
