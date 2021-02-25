import { Waypoint } from './waypoint';
import { Position, Orientation } from './geometry';

export class Location extends Waypoint {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'location.';
    }

    static fullTypeString() {
        return Waypoint.fullTypeString() + Location.typeString();
    }

    constructor(position=null, orientation=null, joints=null, type='', name='', uuid=null, parent=null, appendType=true) {
        super(
            position= position,
            orientation= orientation,
            joints= joints,
            type= (appendType) ? 'location.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType
        );
    }

    static fromDict(dct) {
        return new Location(
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
        //TODO implement this
        return {};
    }
}