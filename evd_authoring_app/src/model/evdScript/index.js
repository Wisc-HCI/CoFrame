import { getEvdCacheObject, Cache } from './cache';

import { Node } from './node';
import { Context } from './context';
import { NodeParser, getExactType } from './utilityFunctions';
import { AttributeTraceProcessor } from './attributeTraceProcessor';

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
    TasksNodeParser,
    Delay, 
    Gripper, 
    MoveTrajectory, 
    MoveUnplanned, 
    PrimitivesNodeParser,
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


const INITIAL_TOOLBOX = {
    contents: [
        {
            kind: "category",
            name: "Locations",
            colour: "260",
            blocks: []
        },
        {
            kind: "category",
            name: "Wayponts",
            colour: "290",
            blocks: []
        },
        {
            kind: "category",
            name: "Trajectories",
            colour: "330",
            blocks: []
        },
        {
            kind: "category",
            name: "Machines",
            colour: "50",
            blocks: []
        },
        {
            kind: "category",
            name: "Things",
            colour: "20",
            blocks: []
        },
        {
            kind: "category",
            name: "Tasks",
            colour: "210",
            blocks: []
        },
        {
            kind: "category",
            name: "Primitives",
            colour: "120",
            blocks: []
        },
        {
            kind: "category",
            name: "Control Flow",
            colour: "160",
            blocks: []
        }
    ]
};

const evdScriptBlocklyToolbox = () => {
    return INITIAL_TOOLBOX.contents;
};


export {
    getEvdCacheObject,
    Cache,
    Node,
    Context,
    NodeParser,
    getExactType,
    AttributeTraceProcessor,
    evdScriptBlocklyToolbox,
    
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
    TasksNodeParser,
    Delay, 
    Gripper, 
    MoveTrajectory, 
    MoveUnplanned, 
    PrimitivesNodeParser,
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