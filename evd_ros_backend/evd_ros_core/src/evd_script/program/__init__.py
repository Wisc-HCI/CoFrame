from .cache import Cache
from .context import Context
from .primitive import Primitive, MoveTrajectory, MoveUnplanned, Delay, Gripper, Breakpoint
from .machine import MachinePrimitive, MachineStart, MachineWait, MachineStop, MachineInitialize, MachineBlockingProcess
from .task import Task, CloseGripper, OpenGripper, PickAndPlace, Initialize
from .flow_control import Loop, Branch
from .program import Program
from .conditionals import *


def ProgramNodeParser(exactType, dct):

    node = ConditionalNodeParser(exactType, dct)
    if node != None:
        return node

    elif exactType == "program":
        node = Program.from_dct(dct)

    elif exactType == "context":
        node = Context.from_dct(dct)

    elif exactType == "task":
        node = Task.from_dct(dct)
    elif exactType == "close-gripper":
        node = CloseGripper.from_dct(dct)
    elif exactType == "open-gripper":
        node = OpenGripper.from_dct(dct)
    elif exactType == "pick-and-place":
        node = PickAndPlace.from_dct(dct)
    elif exactType == "initialize":
        node = Initialize.from_dct(dct)

    elif exactType == "loop":
        node = Loop.from_dct(dct)
    elif exactType == "branch":
        node = Branch.from_dct(dct)

    elif exactType == "machine-primitive":
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

    elif exactType == "primitive":
        node = Primitive.from_dct(dct)
    elif exactType == "move-trajectory":
        node = MoveTrajectory.from_dct(dct)
    elif exactType == "move-unplanned":
        node = MoveUnplanned.from_dct(dct)
    elif exactType == "delay":
        node = Delay.from_dct(dct)
    elif exactType == "gripper":
        node = Gripper.from_dct(dct)
    elif exactType == "breakpoint":
        node = Breakpoint.from_dct(dct)

    return node
