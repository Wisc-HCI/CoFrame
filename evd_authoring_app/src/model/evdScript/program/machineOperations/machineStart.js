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
            machineUuid,
            (appendType) ? 'machine-start'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new MachineStart(
            dct.machine_uuid,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }
}