import { MoveTrajectory } from '../primitives';
import { CloseGripper } from './close_gripper';
import { OpenGripper } from './open_gripper';
import { Task } from '../Task';


export class SimplePickAndPlace extends Task {

    /*
    * Data structure methods
    */

    static typeString() {
        return 'simple-pick-and-place';
    }

    static fullTypeString() {
        return Task.fullTypeString() + SimplePickAndPlace.typeString();
    }

    constructor(startLocUuid=null, pickLocUuid=null, placeLocUuid=null, thingUuid=null, type='', name='', uuid=null, parent=null, appendType=true, primitives=null) {

        if (primitives === null) {
            primitives = [
                MoveTrajectory(startLocUuid,pickLocUuid),
                CloseGripper(thingUuid=thingUuid),
                MoveTrajectory(pickLocUuid,placeLocUuid),
                OpenGripper(thingUuid=thingUuid)
            ];
        }

        super(
            type= (appendType) ? 'simple-pick-and-place.'+type : type,
            name= name,
            uuid= uuid,
            parent= parent,
            appendType= appendType,
            primitives= primitives
        );
    }

    static fromDict(dct) {
        return SimplePickAndPlace(
            type= dct.type,
            name= dct.name,
            uuid= dct.uuid,
            appendType= false,
            primitives= dct.primitives.map(p => NodeParser(p))
        );
    }
}