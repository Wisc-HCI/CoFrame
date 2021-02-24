import { Node } from '../node';
import { Position } from '../data/geometry';


export class PinchPoint extends Node {

    /*
    * Constants
    */

    static GOOD_STATE = "good";
    static WARN_STATE = "warn";
    static ERROR_STATE = "error";

    /*
    * Data structure methods
    */

    static typeString() {
        return 'pinch-point.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + PinchPoint.typeString();
    }

    constructor(axis='x', offset=null, link='', radius=0.5, length=0.2, type='', name='', parent=null, uuid=null, appendType=true) {
        this._axis = null;
        this._offset = null;
        this._link = null;
        this._radius = null;
        this._length = null;

        super(
            type= (appendType) ? 'pinch-point.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.axis = axis;
        this.offset = offset;
        this.link = link;
        this.radius = radius;
        this.length = length;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            axis: this.axis,
            offset: this.offset.toDict(),
            link: this.link,
            radius: this.radius,
            length: this.length
        };
        return msg;
    }

    static fromDict(dct) {
        return PinchPoint(
            axis= dct.axis,
            offset= Position.fromDict(dct.offset),
            link= dct.link,
            radius= dct.radius,
            length= dct.length,
            type= dct.type,
            uuid= dct.uuid,
            name= dct.name,
            appendType= false
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get axis() {
        return this._axis;
    }

    set axis(value) {
        if (this._axis !== value) {
            if (value === null) {
                throw new Error('Axis cannot be null');
            }

            this._axis = value;
            this.updatedAttribute('axis','set');
        }
    }

    get link() {
        return this._link;
    }

    set link(value) {
        if (this._link !== value) {
            this._link = value;
            this.updatedAttribute('link','set');
        }
    }

    get offset() {
        return this._offset;
    }

    set offset(value) {
        if (this._offset !== value) {
            if (value === null) {
                throw new Error('offset cannot be null');
            }

            if (this._offset !== null) {
                this._offset.removeFromCache();
            }
            this._offset = value;
            this._offset.parent = this;
            this.updatedAttribute('offset','set');
        }
    }

    get radius() {
        return this._radius;
    }

    set radius(value) {
        if (this._radius !== value) {
            if (value < 0) {
                throw new Error('Radius must be a positive number or zero');
            }

            this._radius = value;
            this.updatedAttribute('radius','set');
        }
    }

    get length() {
        return this._length;
    }

    set length(value) {
        if (this._length !== value) {
            if (value < 0) {
                throw new Error('Length must be a positive  number or zero');
            }
            this._length = value;
            this.updatedAttribute('length','set');
        }
    }

    set(dct) {

        if ('axis' in dct) {
            this.axis = dct.axis;
        }

        if ('offset' in dct) {
            this.offset = Position.fromDict(dct.offset);
        }

        if ('link' in dct) {
            this.link = dct.link;
        }

        if ('radius' in dct) {
            this.radius = dct.radius;
        }

        if ('length' in dct) {
            this.length = dct.length;
        }

        super.set(dct);
    }

    /*
    * Cache methods
    */

    removeFromCache() {
        this.offset.removeFromCache();
        super.removeFromCache();
    }

    addToCache() {
        this.offset.addToCache();
        super.addToCache();
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        this.offset.lateConstructUpdate();
        super.lateConstructUpdate();
    }

    deepUpdate() {
        this.offset.deepUpdate();
        super.deepUpdate();

        this.updatedAttribute('axis','update');
        this.updatedAttribute('offset','update');
        this.updatedAttribute('link','update');
        this.updatedAttribute('radius','update');
        this.updatedAttribute('length','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('axis','update');
        this.updatedAttribute('offset','update');
        this.updatedAttribute('link','update');
        this.updatedAttribute('radius','update');
        this.updatedAttribute('length','update');
    }
}