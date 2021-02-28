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
        super(
            (appendType) ? 'machine-primitive.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._machineUuid = null;

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
            dct.machine_uuid,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
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