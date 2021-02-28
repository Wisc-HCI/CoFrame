import { MachinePrimitive } from './machinePrimitive';


export class MachineInitialize extends MachinePrimitive {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'machine-initialize';
    }

    static fullTypeString() {
        return MachinePrimitive.fullTypeString() + MachineInitialize.typeString();
    }

    constructor(machineUuid=null, type='', name='', uuid=null, parent=null, appendType=true) {
        super(
            machineUuid,
            (appendType) ? 'machine-initialize'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new MachineInitialize(
            dct.machine_uuid,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

}