import { MachinePrimitive } from './machinePrimitive';

export class MachineStop extends MachinePrimitive {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'machine-stop';
    }

    static fullTypeString() {
        return MachinePrimitive.fullTypeString() + MachineStop.typeString();
    }

    constructor(machineUuid=null, type='', name='', uuid=null, parent=null, appendType=true) {
        super(
            machineUuid,
            (appendType) ? 'machine-stop'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new MachineStop(
            dct.machine_uuid,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

    static BlocklyToolbox() {
        return { type: 'machine_stop' };
    }

    static BlocklyBlock() {
        return { key: 'machine_stop', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Machine Stop");
                this.appendValueInput("machine")
                    .setCheck("machine")
                    .appendField("Machine");
                this.setPreviousStatement(true, null);
                this.setNextStatement(true, null);
                this.setColour(120);
                this.setTooltip("machine-stop");
                this.setHelpUrl("machine-stop");
            }
        }};
    }
}