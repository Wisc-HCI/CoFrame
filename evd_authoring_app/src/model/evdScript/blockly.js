import * as evdData from './data';
import * as evdProgram from './program';

import Blockly from 'blockly';

const {
    Location,
    Waypoint,
    Machine, 
    Trajectory,
    Thing
} = evdData;

const {
    Task,
    Program,
    CloseGripper,
    Initialize,
    OpenGripper,
    SimplePickAndPlace,
    Delay, 
    Gripper, 
    MoveTrajectory, 
    MoveUnplanned, 
    MachineBlockingProcess,
    MachineInitialize,
    MachineStart,
    MachineStop,
    MachineWait,
    Breakpoint, 
    Loop, 
} = evdProgram;


export const evdScriptBlocklyToolbox = () => {
    return [
        {
            kind: "category",
            name: "Locations",
            colour: "260",
            blocks: [
                Location.BlocklyToolbox()
            ]
        },
        {
            kind: "category",
            name: "Wayponts",
            colour: "290",
            blocks: [
                Waypoint.BlocklyToolbox()
            ]
        },
        {
            kind: "category",
            name: "Trajectories",
            colour: "330",
            blocks: [
                Trajectory.BlocklyToolbox()
            ]
        },
        {
            kind: "category",
            name: "Machines",
            colour: "50",
            blocks: [
                Machine.BlocklyToolbox()
            ]
        },
        {
            kind: "category",
            name: "Things",
            colour: "20",
            blocks: [
                Thing.BlocklyToolbox()
            ]
        },
        {
            kind: "category",
            name: "Tasks",
            colour: "210",
            blocks: [
                Task.BlocklyToolbox(),
                Initialize.BlocklyToolbox(),
                CloseGripper.BlocklyToolbox(),
                OpenGripper.BlocklyToolbox(),
                SimplePickAndPlace.BlocklyToolbox(),
                MachineBlockingProcess.BlocklyToolbox()
            ]
        },
        {
            kind: "category",
            name: "Primitives",
            colour: "120",
            blocks: [
                Delay.BlocklyToolbox(),
                Gripper.BlocklyToolbox(),
                MoveTrajectory.BlocklyToolbox(),
                MoveUnplanned.BlocklyToolbox(),
                MachineInitialize.BlocklyToolbox(),
                MachineStart.BlocklyToolbox(),
                MachineWait.BlocklyToolbox(),
                MachineStop.BlocklyToolbox()
            ]
        },
        {
            kind: "category",
            name: "Control Flow",
            colour: "160",
            blocks: [
                Breakpoint.BlocklyToolbox(),
                Loop.BlocklyToolbox()
            ]
        }
    ];
};

export const evdScriptBlocklyInitialize = () => {
    let tmp = null;

    tmp = Location.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Waypoint.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Machine.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Trajectory.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Thing.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Task.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Program.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = CloseGripper.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Initialize.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = OpenGripper.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = SimplePickAndPlace.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Delay.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Gripper.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = MoveTrajectory.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;
    
    tmp = MoveUnplanned.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = MachineBlockingProcess.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = MachineInitialize.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = MachineStart.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = MachineStop.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = MachineWait.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Breakpoint.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;

    tmp = Loop.BlocklyBlock();
    Blockly.Block[tmp.key] = tmp.data;
};

export const evdScriptBlocklyInitialXML = (program = null) => {
    //TODO parse program for initial xml

    let xml = null;
    if (program === null) {
        xml = '<xml xmlns="http://www.w3.org/1999/xhtml"><block type="program></block></xml>';
    }

    return xml;
};