import { Pose, Position, Orientation } from './geometry';


export class Region extends Pose {

    /*
     * Class constants
     */

    static DEFAULT_ORIENTATION_LIMIT = 1;

    /*
    * Data structure methods
    */

    static typeString() {
        return 'region.';
    }

    static fullTypeString() {
        return Pose.fullTypeString() + registerIcons.typeString();
    }

    constructor(centerPosition=null, centerOrientation=null, freeOrientation=true, uncertaintyOrientationLimit=1, uncertaintyOrientationAltTarget=null, type='', name='', uuid=null, parent=null, appendType=true) {
        this._freeOrt = null;
        this._uncertOrtLim = null;
        this._uncertOrtTarget = null;

        super(
            position= centerPosition,
            orientation= centerOrientation,
            type= (appendType) ? 'region.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.freeOrientation = freeOrientation;
        this.uncertaintyOrientationLimit = uncertaintyOrientationLimit;
        this.uncertaintyOrientationAltTarget = uncertaintyOrientationAltTarget;
    }

    toDict() {
        let msg = super.toDict();

        delete msg.position;
        delete msg.orientation;

        msg = {
            ...msg,
            center_position: this.position.toDict(),
            center_orientation: this.orientation.toDict(),
            free_orientation: this.freeOrientation,
            uncertainty_orientation_limit: this.uncertaintyOrientationLimit,
            uncertainty_orientation_alt_target: this.uncertaintyOrientationAltTarget
        }
        return msg;
    }

    static fromDict(dct) {
        return Region(
            centerPosition= Position.fromDict(dct.center_position),
            centerOrientation= Orientation.fromDict(dct.center_orientation),
            freeOrientation= dct.free_orientation,
            uncertaintyOrientationLimit= dct.uncertainty_orientation_limit,
            uncertaintyOrientationAltTarget= Orientation.fromDict(dct.uncertainty_orientation_alt_target),
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false
        );
    }

    toBlockly() {
        //TOOD implement this
        return {};
    }

    /*
    * Data accessor/modifier methods
    */

    get centerPosition() {
        return this.position;
    }

    set centerOrientation(value) {
        this.position = value;
        this.updatedAttribute('center_position','set');
    }

    get centerOrientation() {
        return this.orientation;
    }

    set centerOrientation(value) {
        this.orientation = value;
        this.updatedAttribute('center_orientation','set');
    }

    get freeOrientation() {
        return self._freeOrt;
    }

    set freeOrientation(value) {
        // Note this will destroy previous uncertainty values

        if (this._freeOrt !== value) {
            if (! value instanceof Boolean) {
                throw new Error('Value must be a boolean');
            }

            this._freeOrt = value;
            if (value) {
                this.uncertaintyOrientationLimit = null;
                this.uncertaintyOrientationAltTarget = null;
            } else {
                this.uncertaintyOrientationLimit = Region.DEFAULT_ORIENTATION_LIMIT;
                this.uncertaintyOrientationAltTarget = null;
            }
        }

        this.updatedAttribute('free_orientation','set');
    }

    get uncertaintyOrientationLimit() {
        return this._uncertOrtLim;
    }

    set uncertaintyOrientationLimit(value) {
        if (this._uncertOrtLim !== value) {
            if (this._freeOrt) {
                throw new Error('Orientation is set as free, change freeOrientation to false first!');
            }

            if (value < 0) {
                throw new Error('Value must be positive or zero');
            }

            this._uncertOrtLim = value;
            this.updatedAttribute('uncertainty_orientation_limit','set');
        }
    }

    get uncertaintyOrientationAltTarget() {
        return this._uncertOrtTarget;
    }

    set uncertaintyOrientationAltTarget(value) {
        if (this._uncertOrtTarget !== value) {
            if (this._freeOrt) {
                throw new Error('Orientation is set as free, change freeOrientation to false first!');
            }

            if (this._uncertOrtTarget !== null) {
                this._uncertOrtTarget.removeFromCache();
            }

            this._uncertOrtTarget = value;
            if (this._uncertOrtTarget !== null) {
                this._uncertOrtTarget.parent = this;
            }

            this.updatedAttribute('uncertainty_orientation_alt_target','set');
        }
    }

    set(dct) {

        if ('center_position' in dct) {
            this.centerPosition = Position.fromDict(dct.center_position);
        }

        if ('center_orientation' in dct) {
            this.centerOrientation = Orientation.fromDict(dct.center_orientation);
        }

        if ('free_orientation' in dct) {
            this.freeOrientation = dct.free_orientation;
        }

        if ('uncertainty_orientation_limit' in dct) {
            this.uncertaintyOrientationLimit = dct.uncertainty_orientation_limit;
        }

        if ('uncertainty_orientation_alt_target' in dct) {
            this.uncertaintyOrientationAltTarget = Orientation.fromDict(dct.uncertainty_orientation_alt_target);
        }

        super.set(dct);
    }

    /*
    * Cache Methods 
    */

    removeFromCache() {
        if (this.uncertaintyOrientationAltTarget !== null) {
            this.uncertaintyOrientationAltTarget.removeFromCache();
        }

        super.removeFromCache();
    }

    addToCache() {
        if (this.uncertaintyOrientationAltTarget != null) {
            this.uncertaintyOrientationAltTarget.addToCache();
        }

        super.addToCache();
    }

    /*
    * Update methods
    */

    lateConstructUpdate() {
        if (this.uncertaintyOrientationAltTarget !== null) {
            this.uncertaintyOrientationAltTarget.lateConstructUpdate();
        }

        super.lateConstructUpdate();
    }

    deepUpdate() {

        if (this.uncertaintyOrientationAltTarget !== null) {
            this.uncertaintyOrientationAltTarget.deepUpdate();
        }

        super.deepUpdate();

        this.updatedAttribute('center_position','update');
        this.updatedAttribute('center_orientation','update');
        this.updatedAttribute('free_orientation','update');
        this.updatedAttribute('uncertainty_orientation_limit','update');
        this.updatedAttribute('uncertainty_orientation_alt_target','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('center_position','update');
        this.updatedAttribute('center_orientation','update');
        this.updatedAttribute('free_orientation','update');
        this.updatedAttribute('uncertainty_orientation_limit','update');
        this.updatedAttribute('uncertainty_orientation_alt_target','update');
    }
}

export class CubeRegion extends Region {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'cube-region.';
    }

    static fullTypeString() {
        return Region.fullTypeString() + CubeRegion.type();
    }

    constructor(centerPosition=null, centerOrientation=null, uncertaintyX=1, uncertaintyY=0, uncertaintyZ=0, freeOrientation=true, uncertaintyOrientationLimit=1, uncertaintyOrientationAltTarget=null, type='', name='', uuid=null, parent=null, appendType=true) {
        this._uncertaintyX = null;
        this._uncertaintyY = null;
        this._uncertaintyZ = null;

        super(
            centerPosition= centerPosition,
            centerOrientation= centerOrientation,
            freeOrientation= freeOrientation,
            uncertaintyOrientationLimit= uncertaintyOrientationLimit,
            uncertaintyOrientationAltTarget= uncertaintyOrientationAltTarget,
            type= (appendType) ? 'cube-region.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.uncertaintyX = uncertaintyX;
        this.uncertaintyY = uncertaintyY;
        this.uncertaintyZ = uncertaintyZ;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            uncertainty_x: this.uncertaintyX,
            uncertainty_y: this.uncertaintyY,
            uncertainty_z: this.uncertaintyZ
        };
        return msg;
    }

    static fromDict(dct) {
        return CubeRegion(
            centerPosition= dct.center_position,
            centerOrientation= dct.center_orientation,
            uncertaintyX= dct.uncertainty_x,
            uncertaintyY= dct.uncertainty_y,
            uncertaintyZ= dct.uncertainty_z,
            freeOrientation= dct.free_orientation,
            uncertaintyOrientationLimit= dct.uncertainty_orientation_limit,
            uncertaintyOrientationAltTarget= dct.uncertainty_orientation_alt_target,
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

    get uncertaintyX() {
        return this._uncertaintyX;
    }

    set uncertaintyX(value) {
        if (this._uncertaintyX !== value) {
            this._uncertaintyX = value;
            this.updatedAttribute('uncertainty_x','set');
        }
    }

    get uncertaintyY() {
        return this._uncertaintyY;
    }

    set uncertaintyY(value) {
        if (this._uncertaintyY !== value) {
            this._uncertaintyY = value;
            this.updatedAttribute('uncertainty_y','set');
        }
    }

    get uncertaintyZ() {
        return this._uncertaintyZ;
    }

    set uncertaintyZ(value) {
        if (this._uncertaintyZ !== value) {
            this._uncertaintyZ = value;
            this.updatedAttribute('uncertainty_z','set');
        }
    }

    set(dct) {

        if ('uncertainty_x' in dct) {
            this.uncertaintyX = dct.uncertainty_x;
        }   

        if ('uncertainty_y' in dct) {
            this.uncertaintyY = dct.uncertainty_y;
        }

        if ('uncertainty_z' in dct) {
            this.uncertaintyZ = dct.uncertainty_z;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('uncertainty_x','update');
        this.updatedAttribute('uncertainty_y','update');
        this.updatedAttribute('uncertainty_z','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('uncertainty_x','update');
        this.updatedAttribute('uncertainty_y','update');
        this.updatedAttribute('uncertainty_z','update');
    }
}

export class SphereRegion extends Region {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'sphere-region.';
    }

    static fullTypeString() {
        return Region.fullTypeString() + SphereRegion.type();
    }

    constructor(centerPosition=null, centerOrientation=null, uncertaintyRadius=1, freeOrientation=true, uncertaintyOrientationLimit=1, uncertaintyOrientationAltTarget=null, type='', name='', uuid=null, parent=null, appendType=true) {
        this._uncertaintyRadius = null;

        super(
            centerPosition= centerPosition,
            centerOrientation= centerOrientation,
            freeOrientation= freeOrientation,
            uncertaintyOrientationLimit= uncertaintyOrientationLimit,
            uncertaintyOrientationAltTarget= uncertaintyOrientationAltTarget,
            type= (appendType) ? 'sphere-region.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.uncertaintyRadius = uncertaintyRadius;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            uncertainty_radius: this.uncertaintyRadius,
        };
        return msg;
    }

    static fromDict(dct) {
        return SphereRegion(
            centerPosition= dct.center_position,
            centerOrientation= dct.center_orientation,
            uncertaintyRadius= dct.uncertainty_radius,
            freeOrientation= dct.free_orientation,
            uncertaintyOrientationLimit= dct.uncertainty_orientation_limit,
            uncertaintyOrientationAltTarget= dct.uncertainty_orientation_alt_target,
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

    get uncertaintyRadius() {
        return this._uncertaintyRadius;
    }

    set uncertaintyRadius(value) {
        if (this._uncertaintyRadius !== value) {
            this._uncertaintyRadius = value;
            this.updatedAttribute('uncertainty_radius','set');
        }
    }

    set(dct) {

        if ('uncertainty_radius' in dct) {
            this.uncertaintyRadius = dct.uncertainty_radius;
        }   

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('uncertainty_radius','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('uncertainty_radius','update');
    }
}