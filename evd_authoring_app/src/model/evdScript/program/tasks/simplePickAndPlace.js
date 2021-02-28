import { MoveTrajectory } from '../primitives';
import { CloseGripper } from './closeGripper';
import { OpenGripper } from './openGripper';
import { Task } from '../task';
import { NodeParser } from '../../utilityFunctions';


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

    constructor(startLocUuid=null, pickLocUuid=null, placeLocUuid=null, thingUuid=null, 
                type='', name='', uuid=null, parent=null, appendType=true, primitives=null) 
    {

        if (primitives === null) {
            primitives = [
                MoveTrajectory(startLocUuid,pickLocUuid),
                CloseGripper(thingUuid),
                MoveTrajectory(pickLocUuid,placeLocUuid),
                OpenGripper(thingUuid)
            ];
        }

        super(
            primitives,
            (appendType) ? 'simple-pick-and-place.'+type : type,
            name,
            uuid,
            parent,
            appendType
        );
    }

    static fromDict(dct) {
        return new SimplePickAndPlace(
            null,
            null,
            null,
            null,
            dct.type,
            dct.name,
            dct.uuid,
            null,
            false,
            dct.primitives.map(p => NodeParser(p))
        );
    }
}