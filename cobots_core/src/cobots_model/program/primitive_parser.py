import task
import primitive


def PrimitiveParser(dct):
    exactType = dct['type'].split('.')[-2]

    # Search through tasks
    if exactType == 'task':
        return task.Task.from_dct(dct)
    elif exactType == 'close-gripper':
        return task.CloseGripper.from_dct(dct)
    elif exactType == 'open-gripper':
        return task.OpenGripper.from_dct(dct)
    elif exactType == 'pick-and-place':
        return task.PickAndPlace.from_dct(dct)
    elif exactType == 'initialize':
        return task.Initialize.from_dct(dct)
    elif exactType == 'machine-blocking-process':
        return task.MachineBlockingProcess.from_dct(dct)
    elif exactType == 'loop':
        return task.Loop.from_dct(dct)

    # Search through primitives
    elif exactType == 'primitive':
        return primitive.Primitive.from_dct(dct)
    elif exactType == 'move-trajectory':
        return primitive.MoveTrajectory.from_dct(dct)
    elif exactType == 'move-unplanned':
        return primitive.MoveUnplanned.from_dct(dct)
    elif exactType == 'delay':
        return primitive.Delay.from_dct(dct)
    elif exactType == 'gripper':
        return primitive.Gripper.from_dct(dct)
    elif exactType == 'machine':
        return primitive.Machine.from_dct(dct)
    elif exactType == 'machine-start':
        return primitive.MachineStart.from_dct(dct)
    elif exactType == 'machine-wait':
        return primitive.MachineWait.from_dct(dct)
    elif exactType == 'machine-stop':
        return primitive.MachineStop.from_dct(dct)
    elif exactType == 'machine-initialize':
        return primitive.MachineInitialize.from_dct(dct)
    elif exactType == 'breakpoint':
        return primitive.Breakpoint.from_dct(dct)

    # Could not resolve
    else:
        raise Exception('Could not parse primitive supplied: {}'.format(exactType))
