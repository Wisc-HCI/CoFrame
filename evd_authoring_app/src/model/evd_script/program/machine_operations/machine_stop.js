import { MachinePrimitive } from './machine_primitive';

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
            machineUuid= machineUuid,
            type= (appendType) ? 'machine-stop'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );
    }

    static fromDict(dct) {
        return MachineStop(
            machineUuid= dct.machine_uuid,
            type= dct.type,
            appendType= false,
            name= dct.name,
            uuid= dct.uuid
        );
    }
}