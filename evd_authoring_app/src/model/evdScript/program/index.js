import { Skill } from './skill';
import { Primitive } from './primitive';
import { Program } from './program';

import * as evdSkills from './skills';
import * as evdPrimitives from './primitives';
import * as evdMachineOperations from './machineOperations';
import * as evdControlFlow from './control_flow';


const { 
    CloseGripper,
    Initialize,
    OpenGripper,
    SimplePickAndPlace,
    SkillsNodeParser
} = evdSkills;

const { 
    Delay, 
    Gripper, 
    MoveTrajectory, 
    MoveUnplanned, 
    PrimitivesNodeParser 
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

    node = SkillsNodeParser(exactType,dct);
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
        case 'skill':
            node = Skill.fromDict(dct);
            break;
        case 'primitive':
            node = Primitive.fromDict(dct);
            break;
        case 'program':
            node = Program.fromDict(dct);
            break;
        default:
            break;
    }

    return node;
};

export {
    Skill,
    Primitive,
    Program,
    ProgramNodeParser,

    CloseGripper,
    Initialize,
    OpenGripper,
    SimplePickAndPlace,
    SkillsNodeParser,

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
};