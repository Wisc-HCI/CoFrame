import { CloseGripper } from './close_gripper';
import { Initialize } from './initialize';
import { OpenGripper } from './open_gripper';
import { SimplePickAndPlace } from './simple_pick_and_place';

const TaskNodeParser = (exactType, dct) => {

    let node = null;

    switch(exactType) {
        case 'close-gripper':
            node = CloseGripper.fromDict(dct);
            break;
        case 'open-gripper':
            node = OpenGripper.fromDict(dct);
            break;
        case 'simple-pick-and-place':
            node = SimplePickAndPlace.fromDict(dct);
            break;
        case 'initialize':
            node = Initialize.fromDict(dct);
            break;
    }

    return node;
};

export {
    CloseGripper,
    Initialize,
    OpenGripper,
    SimplePickAndPlace,
    TaskNodeParser
};