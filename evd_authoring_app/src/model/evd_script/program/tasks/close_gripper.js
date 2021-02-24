import { Task } from '../task';
import { Gripper } from '../primitive';
import { NodeParser } from '../../utility_functions';


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

    constructor(position=100, effort=100, speed=100, thingUuid=null, type='', name='', uuid=null, parent=null, appendType=true, primitives=null) {

        if (primitives !== null) {
            primitives = [
                Gripper(
                    thingUuid= thingUuid,
                    position= position,
                    effort= effort,
                    speed= speed
                )
            ];
        }

        super(
            type= (appendType) ? 'close-gripper.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType,
            primitives= primitives
        );
    }

    static fromDict(dct) {
        return CloseGripper(
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false,
            primitives= dct.primitives.map(p => NodeParser(p))
        );
    }
}