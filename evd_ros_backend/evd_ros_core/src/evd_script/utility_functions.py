from .data import *
from .environment import *
from .program import *
from .node import Node


def NodeParser(dct):
    type = dct["type"].split('.')
    exactType = type[len(type) - 2]

    node = None

    if exactType == "program":
        node = Program.FromDct(dct)

    elif exactType == "context":
        node = Context.FromDct(dct)

    elif exactType == "task":
        node = Task.FromDct(dct)
    elif exactType == "close-gripper":
        node = CloseGripper.FromDct(dct)
    elif exactType == "open-gripper":
        node = OpenGripper.FromDct(dct)
    elif exactType == "pick-and-place":
        node = PickAndPlace.FromDct(dct)
    elif exactType == "initialize":
        node = Initialize.FromDct(dct)
    elif exactType =="machine-blocking-process":
        node = MachineBlockingProcess.FromDct(dct)
    elif exactType == "loop":
        node = Loop.FromDct(dct)

    elif exactType == "primitive":
        node = Primitive.FromDct(dct)
    elif exactType == "move-trajectory":
        node = MoveTrajectory.FromDct(dct)
    elif exactType == "move-unplanned":
        node = MoveUnplanned.FromDct(dct)
    elif exactType == "delay":
        node = Delay.FromDct(dct)
    elif exactType == "gripper":
        node = Gripper.FromDct(dct)
    elif exactType == "machine-primitive":
        node = MachinePrimitive.FromDct(dct)
    elif exactType == "machine-start":
        node = MachineStart.FromDct(dct)
    elif exactType == "machine-wait":
        node = MachineWait.FromDct(dct)
    elif exactType == "machine-stop":
        node = MachineStop.FromDct(dct)
    elif exactType == "machine-initialize":
        node = MachineInitialize.FromDct(dct)
    elif exactType == "breakpoint":
        node = Breakpoint.FromDct(dct)

    elif exactType == "reach-sphere":
        node = ReachSphere.FromDct(dct)
    elif exactType == "pinch-point":
        node = PinchPoint.FromDct(dct)
    elif exactType == "collision-mesh":
        node = CollisionMesh.FromDct(dct)
    elif exactType == "occupancy-zone":
        node = OccupancyZone.FromDct(dct)
    elif exactType == "environment":
        node = Environment.FromDct(dct)

    elif exactType == "waypoint":
        node = Waypoint.FromDct(dct)
    elif exactType == "location":
        node = Location.FromDct(dct)
    elif exactType == "machine":
        node = Machine.FromDct(dct)
    elif exactType == "trajectory":
        node = Trajectory.FromDct(dct)
    elif exactType == "trace":
        node = Trace.FromDct(dct)
    elif exactType == "trace-data-point":
        node = TraceDataPoint.FromDct(dct)

    elif exactType == "pose":
        node = Pose.FromDct(dct)
    elif exactType == "position":
        node = Position.FromDct(dct)
    elif exactType == "orientation":
        node = Orientation.FromDct(dct)

    elif exactType == "node":
        node = Node.FromDct(dct)

    else:
        raise Exception('Could not parse object supplied with type: {}'.format(exactType))

    return node


def get_exact_type(node):
    type = node.type.split('.')
    exactType = type[len(type) - 2]
    return exactType
