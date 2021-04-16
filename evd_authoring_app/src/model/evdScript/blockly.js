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
    Skill,
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
            name: "Skills",
            colour: "210",
            blocks: [
                Skill.BlocklyToolbox(),
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
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Waypoint.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Machine.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Trajectory.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Thing.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Skill.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Program.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = CloseGripper.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Initialize.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = OpenGripper.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = SimplePickAndPlace.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Delay.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Gripper.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = MoveTrajectory.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;
    
    tmp = MoveUnplanned.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = MachineBlockingProcess.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = MachineInitialize.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = MachineStart.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = MachineStop.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = MachineWait.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Breakpoint.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Loop.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    console.log(Blockly);
    console.log("Finished initializing custom blocks Blockly");
};

export const evdScriptBlocklyInitialXML = (program = null) => {
    //TODO parse program for initial xml

    let xml = null;
    if (program === null) {
        xml = '<xml xmlns="http://www.w3.org/1999/xhtml"><block type="program" x="30" y="30"></block></xml>';
    }

    return xml;
};