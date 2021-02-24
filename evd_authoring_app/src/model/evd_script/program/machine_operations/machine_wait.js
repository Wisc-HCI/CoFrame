import { MachinePrimitive } from './machine_primitive';

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
            machineUuid= machineUuid,
            type= (appendType) ? 'machine-wait'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );
    }

    static fromDict(dct) {
        return MachineWait(
            machineUuid= dct.machine_uuid,
            type= dct.type,
            appendType= false,
            name= dct.name,
            uuid= dct.uuid
        );
    }
}