import { Node } from '../node';
import { Position } from '../data/geometry';


export class ReachSphere extends Node {

    /*
    * Constants
    */

    static GOOD_STATE = 'good';
    static WARN_STATE = 'warn';
    static ERROR_STATE = 'error';

    /*
    * Data structure methods
    */

    static typeString() {
        return 'reach-sphere.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + ReachSphere.typeString();
    }

    constructor(radius=1, offset=null, type='', name='', parent=null, uuid=null, appendType=true) {
        this._radius = null;
        this._offset = null;

        super(
            type= (appendType) ? 'reach-sphere.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.radius = radius;
        this.offset = (offset !== null) ? offset : new Position(0,0,0);
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            radius: this.radius,
            offset: this.offset.toDict()
        };
        return msg;
    }

    static fromDict(dct) {
        return ReachSphere(
            radius= dct.radius,
            offset= Position.fromDict(dct.offset),
            type= dct.type,
            uuid= dct.uuid,
            name= dct.name,
            appendType= false
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get radius() {
        return this._radius;
    }

    set radius(value) {
        if (this._radius !== value) {
            if (value < 0) {
                throw new Error('Radius must be a positive number');
            }

            this._radius = value;
            this.updatedAttribute('radius','set');
        }
    }

    get offset() {
        return this._offset;
    }

    set offset(value) {
        if (this._offset !== value) {
            if (this._offset !== null) {
                this._offset.removeFromCache();
            }

            this._offset = value;
            this._offset.parent = this;
            this.updatedAttribute('offset','set');
        }
    }

    set(dct) {

        if ('radius' in dct) {
            this.radius = dct.radius;
        }

        if ('offset' in dct) {
            this.offset = Position.fromDict(dct.offset);
        }
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

        this.updatedAttribute('radius','update');
        this.updatedAttribute('offset','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('radius','update');
        this.updatedAttribute('offset','update');
    }
}