import { Task } from '../task';
import { NodeParser } from '../../utilityFunctions';
import { MachineStart } from './machineStart';
import { MachineStop } from './machineStop';
import { MachineWait } from './machineWait';


export class MachineBlockingProcess extends Task {

    /*
    * Data structre methods
    */

    static typeString() {
        return 'machine-blocking-process.';
    }

    static fullTypeString() {
        return Task.fullTypeString() + MachineBlockingProcess.typeString();
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
}