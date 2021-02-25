import { getEvdCacheObject, Cache } from './cache';

import { Node } from './node';
import { Context } from './context';
import { NodeParser, getExactType } from './utility_functions';
import { AttributeTraceProcessor } from './attribute_trace_processor';

import * as evdData from './data';
import * as evdProgram from './program';
import * as evdEnvironment from './environment';
import * as evdTest from './test';

const {
    Pose, 
    Position, 
    Orientation,
    Location,
    Waypoint,
    Machine, 
    MachineRecipe,
    Trace, 
    TraceDataPoint,
    Trajectory,
    Region, 
    CubeRegion, 
    SphereRegion,
    Thing,
    DataNodeParser
} = evdData;

const {
    Task,
    Primitive,
    Program,
    ProgramNodeParser,
    CloseGripper,
    Initialize,
    OpenGripper,
    SimplePickAndPlace,
    TaskNodeParser,
    Delay, 
    Gripper, 
    MoveTrajectory, 
    MoveUnplanned, 
    PrimitiveNodeParser,
    MachineBlockingProcess,
    MachineInitialize,
    MachinePrimitive,
    MachineStart,
    MachineStop,
    MachineWait,
    MachineOperationsNodeParser,
    Branch, 
    Breakpoint, 
    Loop, 
    ControlFlowNodeParser
} = evdProgram;

const {
    EnvironmentNodeParser,
    Environment,
    CollisionMesh,
    OccupancyZone,
    PinchPoint,
    ReachSphere
} = evdEnvironment;

const {
    Container,
    TestNodeParser
} = evdTest;

export {
    getEvdCacheObject,
    Cache,
    Node,
    Context,
    NodeParser,
    getExactType,
    AttributeTraceProcessor,
    
    Pose, 
    Position, 
    Orientation,
    Location,
    Waypoint,
    Machine, 
    MachineRecipe,
    Trace, 
    TraceDataPoint,
    Trajectory,
    Region, 
    CubeRegion, 
    SphereRegion,
    Thing,
    DataNodeParser,

    Task,
    Primitive,
    Program,
    ProgramNodeParser,
    CloseGripper,
    Initialize,
    OpenGripper,
    SimplePickAndPlace,
    TaskNodeParser,
    Delay, 
    Gripper, 
    MoveTrajectory, 
    MoveUnplanned, 
    PrimitiveNodeParser,
    MachineBlockingProcess,
    MachineInitialize,
    MachinePrimitive,
    MachineStart,
    MachineStop,
    MachineWait,
    MachineOperationsNodeParser,
    Branch, 
    Breakpoint, 
    Loop, 
    ControlFlowNodeParser,

    EnvironmentNodeParser,
    Environment,
    CollisionMesh,
    OccupancyZone,
    PinchPoint,
    ReachSphere,

    Container,
    TestNodeParser
};