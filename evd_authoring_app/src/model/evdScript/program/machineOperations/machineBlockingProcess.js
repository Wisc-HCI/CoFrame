import { Task } from '../../task';
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
            type= (appendType) ? 'machine-blocking-process'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType,
            primtiives= primtiives
        );
    }

    static fromDict(dct) {
        return new MachineBlockingProcess(
            type= dct.type,
            appendType= false,
            name= dct.name,
            uuid= dct.uuid,
            primitives= dct.primitives.map(p => NodeParser(p))
        );
    }
}