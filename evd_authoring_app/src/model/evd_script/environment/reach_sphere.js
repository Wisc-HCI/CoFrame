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


}