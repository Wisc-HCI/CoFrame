import { Pose, Position, Orientation } from './geometry';


export class Thing extends Pose {

    /*
    * Class Constants
    */

    static DANGEROUS = 0;
    static SAFE = 1;

    /*
    * Data structure methods
    */

    static typeString() {
        return 'thing.';
    }

    static fullTypeString() {
        return Pose.fullTypeString() + Thing.typeString();
    }

    constructor(thingType, safetyLevel, meshId, position=null, orientation=null, weight=0, type='', name='', parent=null, uuid=null, appendType=true) {
        this._thingType = null;
        this._safetyLevel = null;
        this._meshId = null;
        this._weight = null;

        super(
            position= position,
            orientation= orientation,
            type= (appendType) ? 'thing.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.thingType = thingType;
        this.safetyLevel = safetyLevel;
        this.meshId = meshId;
        this.weigth = weight;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            thing_type: this.thingType,
            safety_level: this.safetyLevel,
            mesh_id: this.meshId,
            weight: this.weight
        };
        return msg;
    }

    static fromDict(dct) {
        return new Thing(
            thingType= dct.thing_type,
            safetyLevel= dct.safety_level,
            meshId= dct.mesh_id,
            position= Position.fromDict(dct.position),
            orientation= Orientation.fromDict(dct.orientation),
            weight= dct.weight,
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false
        );
    }

    toBlockly() {
        // TODO implement this
        return {};
    }

    /*
    * Data accessor/modifier methods
    */

    get thingType() {
        return this._thingType;
    }

    set thingType(value) {
        if (this._thingType !== value) {
            if (value === null) {
                throw new Error('Thing type must exist');
            }

            this._thingType = value;
            this.updatedAttribute('thing_type','set');
        }
    }

    get safetyLevel() {
        return this._safetyLevel;
    }

    set safetyLevel(value) {
        if (this._safetyLevel !== value) {
            if (value > Thing.SAFE || value < Thing.DANGEROUS) {
                throw new Error(`Safety level must be within range (${Thing.DANGEROUS},${Thing.SAFE})`)
            }

            this._safetyLevel = value;
            this.updatedAttribute('safety_level','set');
        }
    }

    get meshId() {
        return this._meshId;
    }

    set meshId(value) {
        if (this._meshId !== value) {
            this._meshId = value;
            this.updatedAttribute('mesh_id','set');
        }
    }

    get weight() {
        return this._weight;
    }

    set weight(value) {
        if (this._weight !== value) {
            this._weight = value;
            this.updatedAttribute('weight','set');
        }
    }

    set(dct) {

        if ('thing_type' in dct) {
            this.thingType = dct.thing_type;
        }

        if ('safety_level' in dct) {
            this.safetyLevel = dct.safety_level;
        }

        if ('mesh_id' in dct) {
            this.meshId = dct.mesh_id;
        }

        if ('weight' in dct) {
            this.weight = dct.weight;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('thing_type','update');
        this.updatedAttribute('safety_level','set');
        this.updatedAttribute('mesh_id','update');
        this.updatedAttribute('weight','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('thing_type','update');
        this.updatedAttribute('safety_level','set');
        this.updatedAttribute('mesh_id','update');
        this.updatedAttribute('weight','update');
    }
}