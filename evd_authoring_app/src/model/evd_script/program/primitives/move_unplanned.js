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
        this._locationUuid = null;
        this._manualSafety = null;

        super(
            type= (appendType) ? 'move-unplanned.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

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
            name= dct.name,
            type= dct.type,
            appendType= false,
            uuid= dct.uuid,
            locUuid= dct.location_uuid,
            manualSafety= dct.manual_safety
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