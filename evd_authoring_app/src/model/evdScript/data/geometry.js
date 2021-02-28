
import { Node } from '../node';

export class Pose extends Node {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'pose.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + Pose.typeString();
    }

    constructor(position=null, orientation=null, type='', name='', uuid=null, parent=null, appendType=true) {
        super(
            (appendType) ? 'pose.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._position = null;
        this._orientation = null;

        this.position = (position !== null) ? position : new Position(0,0,0);
        this.orientation = (orientation !== null) ? orientation : new Orientation(0,0,0,1);
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            position: this.position.toDict(),
            orientation: this.orientation.toDict()
        };
        return msg;
    }

    static fromDict(dct) {
        return new Pose(
            Position.fromDict(dct.position),
            Orientation.fromDict(dct.orientation),
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

    get position() {
        return this._position;
    }

    set position(value) {
        if (this._position !== value) {
            if (value === null || value === undefined) {
                throw new Error('Position cannot be null or undefined');
            }

            if (this._position !== null) {
                this._position.removeFromCache();
            }

            this._position = value;
            this._position.parent = this;
            this.updatedAttribute('position','set');
        }
    }

    get orientation() {
        return this._orientation;
    }

    set orientation(value) {
        if (this._orientation !== value) {
            if (value === null || value === undefined) {
                throw new Error('Orienation cannot be null or undefined');
            }

            if (this._orientation !== null) {
                this._orientation.removeFromCache();
            }

            this._orientation = value;
            this._orientation.parent = this;
            this.updatedAttribute('orienation','set');
        }
    }

    set(dct) {

        if ('position' in dct) {
            this.position = Position.fromDict(dct.position);
        }

        if ('orientation' in dct) {
            this.orientation = Orientation.fromDict(dct.orientation);
        }

        super.set(dct);
    }

    /*
    * Cache method
    */

    removeFromCache() {
        this.position.removeFromCache();
        this.orientation.removeFromCache();

        super.removeFromCache();
    }

    addToCache() {
        this.position.addToCache();
        this.orientation.addToCache();

        super.addToCache();
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        this.position.lateConstructUpdate();
        this.orientation.lateConstructUpdate();

        super.lateConstructUpdate();
    }

    deepUpdate() {
        this.position.deepUpdate();
        this.orientation.deepUpdate();

        super.deepUpdate();

        this.updatedAttribute('position','update');
        this.updatedAttribute('orientation','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('position','update');
        this.updatedAttribute('orientation','update');
    }
}

export class Position extends Node {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'position.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + Position.typeString();
    }

    constructor(x, y, z, type='', name='', parent=null, uuid=null, appendType=true) {
        super(
            (appendType) ? 'position.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._x = null;
        this._y = null;
        this._z = null;

        this.x = x;
        this.y = y;
        this.z = z;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            x: this.x,
            y: this.y,
            z: this.z
        };
        return msg;
    }

    static fromDict(dct) {
        return new Position(
            dct.x,
            dct.y,
            dct.z,
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

    get x() {
        return this._x;
    }

    set x(value) {
        if (this._x !== value) {
            this._x = value;
            this.updatedAttribute('x','set');
        }
    }

    get y() {
        return this._y;
    }

    set y(value) {
        if (this._y !== value) {
            this._y = value;
            this.updatedAttribute('y','set');
        }
    }

    get z() {
        return this._x;
    }

    set z(value) {
        if (this._z !== value) {
            this._z = value;
            this.updatedAttribute('z','set');
        }
    }

    set(dct) {

        if ('x' in dct) {
            this.x = dct.x;
        }

        if ('y' in dct) {
            this.y = dct.y;
        }

        if ('z' in dct) {
            this.z = dct.z;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('x','update');
        this.updatedAttribute('y','update');
        this.updatedAttribute('z','update');
    }

    shallowUpdate() {
        super.deepUpdate();

        this.updatedAttribute('x','update');
        this.updatedAttribute('y','update');
        this.updatedAttribute('z','update');
    }

}

export class Orientation extends Node {

        /*
        * Data structure methods
        */

    static typeString() {
        return 'orientation.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + Orientation.typeString();
    }

    constructor(x, y, z, w, type='', name='', parent=null, uuid=null, appendType=true) {
        super(
            (appendType) ? 'orientation.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );

        this._x = null;
        this._y = null;
        this._z = null;
        this._w = null;

        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            x: this.x,
            y: this.y,
            z: this.z,
            w: this.w
        };
        return msg;
    }

    static fromDict(dct) {
        return new Position(
            dct.x,
            dct.y,
            dct.z,
            dct.w,
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

    get x() {
        return this._x;
    }

    set x(value) {
        if (this._x !== value) {
            this._x = value;
            this.updatedAttribute('x','set');
        }
    }

    get y() {
        return this._y;
    }

    set y(value) {
        if (this._y !== value) {
            this._y = value;
            this.updatedAttribute('y','set');
        }
    }

    get z() {
        return this._x;
    }

    set z(value) {
        if (this._z !== value) {
            this._z = value;
            this.updatedAttribute('z','set');
        }
    }

    get w() {
        return this._w;
    }

    set w(value) {
        if (this._w !== value) {
            this._w = value;
            this.updatedAttribute('w','set');
        }
    }

    set(dct) {

        if ('x' in dct) {
            this.x = dct.x;
        }

        if ('y' in dct) {
            this.y = dct.y;
        }

        if ('z' in dct) {
            this.z = dct.z;
        }

        if ('w' in dct) {
            this.w = dct.w;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('x','update');
        this.updatedAttribute('y','update');
        this.updatedAttribute('z','update');
        this.updatedAttribute('w','update');
    }

    shallowUpdate() {
        super.deepUpdate();

        this.updatedAttribute('x','update');
        this.updatedAttribute('y','update');
        this.updatedAttribute('z','update');
        this.updatedAttribute('w','update');
    }

}