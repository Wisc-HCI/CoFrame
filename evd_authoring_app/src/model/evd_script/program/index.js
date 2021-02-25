import { Task } from './task';
import { Primitive } from './primitive';
import { Program } from './program';

import * as evdTasks from './tasks';
import * as evdPrimitives from './primitives';
import * as evdMachineOperations from './machine_operations';
import * as evdControlFlow from './control_flow';


const { 
    CloseGripper,
    Initialize,
    OpenGripper,
    SimplePickAndPlace,
    TaskNodeParser
} = evdTasks;

const { 
    Delay, 
    Gripper, 
    MoveTrajectory, 
    MoveUnplanned, 
    PrimitiveNodeParser 
} = evdPrimitives;

const {
    MachineBlockingProcess,
    MachineInitialize,
    MachinePrimitive,
    MachineStart,
    MachineStop,
    MachineWait,
    MachineOperationsNodeParser
} = evdMachineOperations;

const {
    Branch, 
    Breakpoint, 
    Loop, 
    ControlFlowNodeParser 
} = evdControlFlow;


const ProgramNodeParser = (exactType, dct) => {

    let node = null;

    node = TasksNodeParser(exactType,dct);
    if (node !== null) {
        return node;
    }

    node = PrimitivesNodeParser(exactType,dct);
    if (node !== null) {
        return node;
    }

    node = MachineOperationsNodeParser(exactType,dct);
    if (node !== null) {
        return node;
    }

    node = ControlFlowNodeParser(exactType,dct);
    if (node !== null) {
        return node;
    }

    switch(exactType) {
        case 'task':
            node = Task.fromDict(dct);
            break;
        case 'primitive':
            node = Primitive.fromDict(dct);
            break;
        case 'program':
            node = Program.fromDict(dct);
            break;
    }

    return node;
};





export {
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
};