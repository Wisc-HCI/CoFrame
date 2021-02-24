import { Task } from '.task';
import { Primitive } from '.primitive';
import { Program } from './program';

import * as evdTasks from './tasks';
import * as evdPrimitives from './primitives';
import * as evdMachineOperations from './machine_operations';
import * as evdControlFlow from './control_flow';


const ProgramNodeParser = (exactType, dct) => {

    let node = null;

    node = evdTasks.TasksNodeParser(exactType,dct);
    if (node !== null) {
        return node;
    }

    node = evdPrimitives.PrimitivesNodeParser(exactType,dct);
    if (node !== null) {
        return node;
    }

    node = evdMachineOperations.MachineOperationsNodeParser(exactType,dct);
    if (node !== null) {
        return node;
    }

    node = evdControlFlow.ControlFlowNodeParser(exactType,dct);
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
    evdTasks,
    evdPrimitives,
    evdMachineOperations,
    evdControlFlow,
    ProgramNodeParser
};