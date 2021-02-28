import { Task } from '../task';
import { Gripper } from '../primitives';
import { NodeParser } from '../../utilityFunctions';


export class OpenGripper extends Task {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'open-gripper';
    }

    static fullTypeString() {
        return Task.fullTypeString() + OpenGripper.typeString();
    }

    constructor(thingUuid=null, position=0, effort=100, speed=100, type='', name='', 
                uuid=null, parent=null, appendType=true, primitives=null) 
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
            (appendType) ? 'open-gripper.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new OpenGripper(
            null,
            0,
            100,
            100,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false,
            dct.primitives.map(p => NodeParser(p))
        );
    }
}