import { Node } from '../node';
import { Pose, Position } from '../data/geometry';


export class OccupancyZone extends Node {

    /*
    * Class Constants
    */

    static HUMAN_TYPE = 'human';
    static ROBOT_TYPE = 'robot';

    /*
    * Data structure methods
    */

    static typeString() {
        return 'occupancy-zone.';
    }

    static fullTypeString() {
        return Node.fullTypeString() + OccupancyZone.typeString();
    }

    constructor(occupancyType, posX=0, posZ=0, sclX=1, sclZ=1, height=0, type='', name='', parent=null, uuid=null, appendType=true) {
        this._occupancyType = null;
        this._positionX = null;
        this._positionZ = null;
        this._scaleX = null;
        this._scaleZ = null;
        this._height = null;

        super(
            type= (appendType) ? 'occupancy-zone.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.occupancyType = occupancyType;
        this.positionX = posX;
        this.positionZ = posZ;
        this.scaleX = sclX;
        this.scaleZ = sclZ;
        this.height = height;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            occupancy_type: this.occupancyType,
            position_x: this.positionX,
            position_z: this.positionZ,
            scale_x: this.scaleX,
            scale_z: this.scaleZ,
            height: this.height
        };
        return msg;
    }

    static fromDict(dct) {
        return OccupancyZone(
            occupancyType= dct.occupancy_type,
            posX= dct.position_x,
            posZ= dct.position_z,
            sclX= dct.scale_x,
            sclZ= dct.scale_z,
            height= dct.height,
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false
        );
    }

    /*
    * Data accessor/modifier methods
    */

    get occupancyType() {
        return this._occupancyType;
    }

    set occupancyType(value) {
        if (this._occupancyType !== value) {
            if (value !== this.ROBOT_TYPE || value !== this.HUMAN_TYPE) {
                throw new Error('Invalid occupancy zone type specified');
            }

            this._occupancyType = value;
            this.updatedAttribute('occupancy_type','set');
        }
    }

    get positionX() {
        return this._positionX;
    }

    set positionX(value) {
        if (this._positionX !== value) {
            this._positionX = value;
            this.updatedAttribute('position_x','set');
        }
    }

    get positionZ() {
        return this._positionZ;
    }

    set positionZ(value) {
        if (this._positionZ !== value) {
            this._positionZ = value;
            this.updatedAttribute('position_z','set');
        }
    }

    get scaleX() {
        return this._scaleX;
    }

    set scaleX(value) {
        if (this._scaleX !== value) {
            this._scaleX = value;
            this.updatedAttribute('scale_x','set');
        }
    }

    get scaleZ() {
        return this._scaleZ;
    }

    set scaleZ(value) {
        if (this._scaleZ !== value) {
            this._scaleZ = value;
            this.updatedAttribute('scale_z','set');
        }
    }

    get height() {
        return this._height;
    }

    set height(value) {
        if (this._height !== value) {
            this._height = value;
            this.updatedAttribute('height','set');
        }
    }

    set(dct) {

        if ('occupancy_type' in dct) {
            this.occupancyType = dct.occupancy_type;
        }

        if ('position_x' in dct) {
            this.positionX = dct.position_x;
        }

        if ('position_z' in dct) {
            this.positionZ = dct.position_z;
        }   

        if ('scale_x' in dct) {
            this.scaleX = dct.scale_x;
        } 

        if ('scale_z' in dct) {
            this.scaleZ = dct.scale_z;
        }

        if ('height' in dct) {
            this.height = dct.height;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('occupancy_type','update');
        this.updatedAttribute('position_x','update');
        this.updatedAttribute('position_z','update');
        this.updatedAttribute('scale_x','update');
        this.updatedAttribute('scale_z','update');
        this.updatedAttribute('height','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('occupancy_type','update');
        this.updatedAttribute('position_x','update');
        this.updatedAttribute('position_z','update');
        this.updatedAttribute('scale_x','update');
        this.updatedAttribute('scale_z','update');
        this.updatedAttribute('height','update');
    }
}