import { OpenGripper } from '.';
import { NodeParser } from '../../utilityFunctions';
import { MachineInitialize } from '../machineOperations';
import { MoveUnplanned } from '../primitives';
import { Task } from '../task';


export class Initialize extends Task {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'initialize.';
    }

    static fullTypeString() {
        return Task.fullTypeString() + Initialize.typeString();
    }

    constructor(homeLocUuid=null, machineUuids=[], type='', name='', uuid=null, 
                parent=null, appendType=true, primitives=null) 
    {

        if (primitives === null) {
            primitives = [];

            primitives = primitives + machineUuids.map(id => MachineInitialize(id));

            primitives = primitives + [
                MoveUnplanned(homeLocUuid,true),
                OpenGripper()
            ];
        }

        super(
            primitives,
            type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new Initialize(
            null,
            [],
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false,
            dct.primitives.map(p => NodeParser(p))
        );
    }

    static BlocklyToolbox() {
        return { type: 'initialize' };
    }

    static BlocklyBlock() {
        return { key: 'initialize', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Initialize");
                this.appendValueInput("home-location")
                    .setCheck(null)
                    .appendField("Home Location");
                this.setPreviousStatement(true, null);
                this.setNextStatement(true, null);
                this.setColour(210);
                this.setTooltip("initialize");
                this.setHelpUrl("initialize");
            }
        }};
    }
}