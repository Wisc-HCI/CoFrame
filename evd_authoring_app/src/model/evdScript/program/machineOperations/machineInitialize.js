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
            machineUuid= machineUuid,
            type= (appendType) ? 'machine-initialize'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );
    }

    static fromDict(dct) {
        return new MachineInitialize(
            machineUuid= dct.machine_uuid,
            type= dct.type,
            appendType= false,
            name= dct.name,
            uuid= dct.uuid
        );
    }

}