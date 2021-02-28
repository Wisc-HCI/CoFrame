import { Task } from '../task';
import { Gripper } from '../primitives';
import { NodeParser } from '../../utilityFunctions';


export class CloseGripper extends Task {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'close-gripper';
    }

    static fullTypeString() {
        return Task.fullTypeString() + CloseGripper.typeString();
    }

    constructor(thingUuid=null, position=100, effort=100, speed=100, 
                type='', name='', uuid=null, parent=null, appendType=true, 
                primitives=null) 
    {

        if (primitives !== null) {
            primitives = [
                Gripper(
                    thingUuid,
                    position,
                    effort,
                    speed
                )
            ];
        }

        super(
            primitives,
            (appendType) ? 'close-gripper.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new CloseGripper(
            null,
            100,
            100,
            100,
            dct.type,
            dct.name,
            dct.uuid,
            false,
            dct.primitives.map(p => NodeParser(p))
        );
    }
}