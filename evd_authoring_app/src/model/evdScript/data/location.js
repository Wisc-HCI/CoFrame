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

    constructor(position=null, orientation=null, joints=null, type='', name='', 
                uuid=null, parent=null, appendType=true) 
    {
        super(
            position,
            orientation,
            joints,
            (appendType) ? 'location.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new Location(
            Position.fromDict(dct.position),
            Orientation.fromDict(dct.orientation),
            dct.joints,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false
        );
    }

    toBlockly() {
        //TODO implement this
        return {};
    }
}