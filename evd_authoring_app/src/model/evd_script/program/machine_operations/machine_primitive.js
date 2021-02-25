import { Primitive } from '../primitive';

export class MachinePrimitive extends Primitive {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'machine-primitive';
    }

    static fullTypeString() {
        return Primitive.fullTypeString() + MachinePrimitive.typeString();
    }

    constructor(machineUuid=null, type='', name='', uuid=null, parent=null, appendType=true) {
        this._machineUuid = null;

        super(
            type= (appendType) ? 'machine-primitive.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.machineUuid = machineUuid;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            machine_uuid: this.machineUuid
        };
        return msg;
    }

    static fromDict(dct) {
        return new MachinePrimitive(
            machineUuid= dct.machine_uuid,
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get machineUuid() {
        return this._machineUuid;
    }

    set machineUuid(value) {
        if (this._machineUuid !== value) {
            this._machineUuid = value;
            this.updatedAttribute('machine_uuid','set');
        }
    }

    set(dct) {

        if ('machine_uuid' in dct) {
            this.machineUuid = dct.machine_uuid;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('machine_uuid','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('machine_uuid','update');
    }
}