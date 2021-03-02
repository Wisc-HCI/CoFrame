import { MachinePrimitive } from './machinePrimitive';

export class MachineWait extends MachinePrimitive {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'machine-wait';
    }

    static fullTypeString() {
        return MachinePrimitive.fullTypeString() + MachineWait.typeString();
    }

    constructor(machineUuid=null, type='', name='', uuid=null, parent=null, appendType=true) {
        super(
            machineUuid,
            (appendType) ? 'machine-wait'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new MachineWait(
            dct.machine_uuid,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

    static BlocklyToolbox() {
        return { type: 'machine_wait' };
    }

    static BlocklyBlock() {
        return { key: 'machine_wait', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Machine Wait");
                this.appendValueInput("machine")
                    .setCheck("machine")
                    .appendField("Machine");
                this.setPreviousStatement(true, null);
                this.setNextStatement(true, null);
                this.setColour(120);
                this.setTooltip("machine-wait");
                this.setHelpUrl("machine-wait");
            }
        }};
    }
}