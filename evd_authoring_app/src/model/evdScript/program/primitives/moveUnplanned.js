import { Primitive } from '../primitive';

export class MoveUnplanned extends Primitive {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'move-unplanned.';
    }

    static fullTypeString() {
        return Primitive.fullTypeString() + MoveUnplanned.typeString();
    }

    constructor(locUuid, manualSafety=true, type='', name='', uuid=null, parent=null, appendType=true) {
        super(
            (appendType) ? 'move-unplanned.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._locationUuid = null;
        this._manualSafety = null;

        this.manualSafety = manualSafety;
        this.locationUuid = locUuid;
    }

    toDict() {
        const msg  = {
            ...super.toDict(),
            location_uuid: this.locationUuid,
            manual_safety: this.manualSafety
        };
        return msg;
    }

    static fromDict(dct) {
        return new MoveUnplanned(
            dct.location_uuid,
            dct.manual_safety,
            dct.name,
            dct.type,
            dct.uuid,
            null,
            false
        );
    }

    /*
    * Data accessor / modifier methods
    */

    get manualSafety() {
        return this._manualSafety;
    }

    set manualSafety(value) {
        if (this._manualSafety !== value) {
            this._manualSafety = value;
            this.updatedAttribute('manual_safety','set');
        }
    }

    get locationUuid() {
        return this._locationUuid;
    }

    set locationUuid(value) {
        if (this._locationUuid !== value) {
            this._locationUuid = value;
            this.updatedAttribute('location_uuid','set');
        }
    }

    set(dct) {
        if ('location_uuid' in dct) {
            this.locationUuid = dct.location_uuid;
        }

        if ('manual_safety' in dct) {
            this.manualSafety = dct.manual_safety;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('location_uuid','update');
        this.updatedAttribute('manaul_safety','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('location_uuid','update');
        this.updatedAttribute('manual_safety','update');
    }

}