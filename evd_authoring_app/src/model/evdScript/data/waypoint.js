import { Pose, Position, Orientation } from './geometry';


export class Waypoint extends Pose {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'waypoint.';
    }

    static fullTypeString() {
        return Pose.fullTypeString() + Waypoint.typeString();
    }

    constructor(position=null, orientation=null, joints=null, type='', name='', uuid=null, parent=null, appendType=true) {
        this._joints = null;

        super(
            position= position,
            orientation= orientation,
            type= (appendType) ? 'waypoint.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );

        this.joints = joints;
    }

    toDict() {
        const msg = {
            ...super.toDict(),
            joints: this.joints
        };
        return msg;
    }

    static fromDict(dct) {
        return new Waypoint(
            position= Position.fromDict(dct.position),
            orientation= Orientation.fromDict(dct.orientation),
            joints= dct.joints,
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

    get joints() {
        return this._joints;
    }

    set joints(value) {
        if (this._joints !== value) {
            this._joints = value;
            this.updatedAttribute('joints','set');
        }
    }

    set(dct) {

        if ('joints' in dct) {
            this.joints = dct.joints;
        }

        super.set(dct);
    }

    /*
    * Update methods
    */

    deepUpdate() {
        super.deepUpdate();

        this.updatedAttribute('joints','update');
    }

    shallowUpdate() {
        super.shallowUpdate();

        this.updatedAttribute('joints','update');
    }

}