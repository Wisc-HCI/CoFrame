import { Skill } from '../skill';
import { NodeParser } from '../../utilityFunctions';
import { MachineStart } from './machineStart';
import { MachineStop } from './machineStop';
import { MachineWait } from './machineWait';


export class MachineBlockingProcess extends Skill {

    /*
    * Data structre methods
    */

    static typeString() {
        return 'machine-blocking-process.';
    }

    static fullTypeString() {
        return Skill.fullTypeString() + MachineBlockingProcess.typeString();
    }

    constructor(machineUuid=null, type='', name='', uuid=null, parent=null, appendType=true, primtiives=null) {

        if (primtiives === null) {
            primtiives = [
                MachineStart(machineUuid),
                MachineWait(machineUuid),
                MachineStop(machineUuid)
            ];
        }

        super(
            primtiives,
            (appendType) ? 'machine-blocking-process'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new MachineBlockingProcess(
            null,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false,
            dct.primitives.map(p => NodeParser(p))
        );
    }

    static BlocklyToolbox() {
        return { type: 'machine_blocking_process' };
    }

    static BlocklyBlock() {
        return { key: 'machine_blocking_process', data: {
            init: function() {
                this.appendDummyInput()
                    .appendField("Machine Blocking Process");
                this.appendValueInput("machine")
                    .setCheck(null)
                    .appendField("Machine");
                this.setPreviousStatement(true, null);
                this.setNextStatement(true, null);
                this.setColour(210);
                this.setTooltip("machine-blocking-process");
                this.setHelpUrl("machine-blocking-process");
            }
        }};
    }
}