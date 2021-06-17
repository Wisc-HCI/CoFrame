import * as evdData from './data';
import * as evdProgram from './program';

// import Blockly from 'blockly';

import Blockly from 'node-blockly/browser';

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
    // Checking how lists are built when adding items afterwards
    var locs = [["loc-1", "loc-1"], ["loc-2", "loc-2"], ["loc-3", "loc-3"]];
    var someblocks = Location.BlocklyBlock(locs);
    locs.push(["loc-4", "loc-4"])

    return [
        // Blocks for testing/tutorials, learning Blockly.
        {
            name: 'HelloWorld',
            category: 'Testing',
            block: {
                init: function () {
                this.jsonInit({
                    message0: 'Hello %1',
                    args0: [
                    {
                        type: 'field_input',
                        name: 'NAME',
                        check: 'String',
                    },
                    // accept a location
                    ],
                    output: 'String',
                    tooltip: 'Says Hello',
                });
                },
            },
            generator: (block) => {
                const message = `'${block.getFieldValue('NAME')}'` || '\'\'';
                const code = `console.log('Hello ${message}')`;
                return [code, Blockly.JavaScript.ORDER_MEMBER];
            },
        },
        {
            name: 'legnth_block',
            category: 'Testing',
            block: {
                init: function () {
                    this.jsonInit({
                        type: "length_block",
                        message0: "length of %1",
                        args0: [
                            {
                                type: "input_value",
                                name: "FROM",
                                check: "String"
                            }
                        ],
                        output: "Number",
                        tooltip: "testing tooltip",
                        helpUrl: ""
                    });
                },
            },
            generator: (block) => {
                var value_length = Blockly.JavaScript.valueToCode(block, 'FROM', Blockly.JavaScript.ORDER_ATOMIC) || '\'\'';
                // TODO: Assemble JavaScript into code variable.
                var code = `console.log('Length is: ${value_length.length}')`;
                return [code, Blockly.JavaScript.ORDER_MEMBER];
            },
        },
        {
            name: 'string_msg',
            category: 'Testing',
            block: {
                init: function () {
                    this.jsonInit({
                        type: "string_msg",
                        message0: 'String: %1',
                        args0: [
                            {
                                type: 'field_input',
                                name: 'VALUE',
                                check: 'String',
                            },
                        ],
                        output: 'String',
                        tooltip: 'Says Hello',
                    });
                },
            },
            generator: (block) => {
                return [block.getFieldValue('VALUE') || '\'\'', Blockly.JavaScript.ORDER_MEMBER]
            }
        },
        {
            name: 'number_msg',
            category: 'Testing',
            block: {
                init: function () {
                    this.jsonInit({
                        type: "number_msg",
                        message0: 'Number: %1',
                        args0: [
                            {
                                type: 'field_input',
                                name: 'VALUE',
                                check: 'Number',
                            },
                        ],
                        output: 'Number',
                        tooltip: 'Says Hello',
                    });
                },
            },
            generator: (block) => {
                return [block.getFieldValue('VALUE') || '0', Blockly.JavaScript.ORDER_MEMBER]
            }
        },

        Machine.BlocklyBlock([["machine-1", "machine-1"], ["machine-2", "machine-2"], ["machine-3", "machine-3"]]),

        // Locations, testing
        someblocks,

        // TODO
        // Waypoint.BlocklyToolbox()
        // colour: "290",

        // Trajectories.BlocklyToolbox()
        // colour: "330"
        
        // Thing.BlocklyToolbox()
        // colour: "20"

        // ----------Skills
        // Skill.BlocklyToolbox(),
        // Initialize.BlocklyToolbox(),
        // CloseGripper.BlocklyToolbox(),
        // OpenGripper.BlocklyToolbox(),
        SimplePickAndPlace.BlocklyBlock(),
        // MachineBlockingProcess.BlocklyToolbox()
        // colour: "210",

        // ----------Primitives
        // Delay.BlocklyToolbox(),
        // Gripper.BlocklyToolbox(),
        // MoveTrajectory.BlocklyToolbox(),
        // MoveUnplanned.BlocklyToolbox(),
        // MachineInitialize.BlocklyToolbox(),
        // MachineStart.BlocklyToolbox(),
        // MachineWait.BlocklyToolbox(),
        // MachineStop.BlocklyToolbox()
        // colour: "120"

        // ----------Control Flow
        // Breakpoint.BlocklyToolbox(),
        // Loop.BlocklyToolbox()
        // colour: "160"
    ];
};

export const evdScriptBlocklyInitialize = () => {
    let tmp = null;

    const locations = []
    tmp = Location.BlocklyBlock(locations); //needs to take in list of location options
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Waypoint.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Machine.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Trajectory.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Thing.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    // THis should generate a skill template block (like program)
    // and should generate an instance block (or blocks)
    // instance of skill should not be parameterized
    tmp = Skill.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    tmp = Program.BlocklyBlock();
    Blockly.Blocks[tmp.key] = tmp.data;

    // Maybe revist parameterization here
    //  For learning over time - maybe switching this to instance & template model
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
        xml = '<xml><block type="program" deletable="false"></block></xml>';
    }

    return xml;
};