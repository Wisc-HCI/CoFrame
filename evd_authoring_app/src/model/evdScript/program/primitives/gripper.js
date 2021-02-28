import { Primitive } from '../primitive';

export class Gripper extends Primitive {

    /**
     * data structure methods
     */

    static typeString() {
        return 'gripper.';
    }

    static fullTypeString() {
        return Primitive.fullTypeString() + Gripper.typeString();
    }

    constructor(thingUuid=null, position=0, effort=0, speed=0, type='', name='', 
                uuid=null, parent=null, appendType=true) 
    {
        super(
            (appendType) ? 'gripper.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._thingUuid = null;
        this._position = null;
        this._effort = null;
        this._speed = null;

        this.thingUuid = thingUuid;
        this.position = position;
        this.effort = effort;
        this.speed = speed;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            thing_uuid: this.thingUuid,
            position: this.position,
            effort: this.effort,
            speed: this.speed
        };
        return msg;
    }

    static fromDict(dct) {
        return new Gripper(
            dct.thing_uuid,
            dct.position,
            dct.effort,
            dct.speed,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

     /**
      * data accessor / modifier methods
      */
    
    get position() {
        return this._position;
    }

    set position(value) {
        if (this._position !== value) {
            this._position = value;
            this.updatedAttribute('position','set');
        }
    }

    get effort() {
        return this._effort;
    }

    set effort(value) {
        if (this._effort !== value) {
            this._effort = value;
            this.updatedAttribute('effort','set');
        }
    }

    get speed() {
        return this._speed;
    }

    set speed(value) {
        if (this._speed !== value) {
            this._speed = value;
            this.updatedAttribute('speed','set');
        }
    }

    get thingUuid() {
        return this._thingUuid;
    }
    
    set thingUuid(value) {
        if (this._thingUuid !== value) {
            this._thingUuid = value;
            this.updatedAttribute('thing_uuid','set');
        }
    }

    set(dct) {

        if ('position' in dct) {
            this.position = dct.position;
        }

        if ('effort' in dct) {
            this.effort = dct.effort;
        }

        if ('speed' in dct) {
            this.speed = dct.speed;
        }

        if ('thing_uuid' in dct) {
            this.thingUuid = dct.thing_uuid;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('thing_uuid','update');
        this.updatedAttribute('position','update');
        this.updatedAttribute('effort','update');
        this.updatedAttribute('speed','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('thing_uuid','update');
        this.updatedAttribute('position','update');
        this.updatedAttribute('effort','update');
        this.updatedAttribute('speed','update');
    }
}