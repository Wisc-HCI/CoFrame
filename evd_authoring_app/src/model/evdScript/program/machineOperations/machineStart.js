import { MachinePrimitive } from './machinePrimitive';

export class MachineStart extends MachinePrimitive {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'machine-start';
    }

    static fullTypeString() {
        return MachinePrimitive.fullTypeString() + MachineStart.typeString();
    }

    constructor(machineUuid=null, type='', name='', uuid=null, parent=null, appendType=true) {
        super(
            machineUuid= machineUuid,
            type= (appendType) ? 'machine-start'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );
    }

    static fromDict(dct) {
        return new MachineStart(
            machineUuid= dct.machine_uuid,
            type= dct.type,
            appendType= false,
            name= dct.name,
            uuid= dct.uuid
        );
    }
}