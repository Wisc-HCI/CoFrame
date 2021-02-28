import { CloseGripper } from './closeGripper';
import { Initialize } from './initialize';
import { OpenGripper } from './openGripper';
import { SimplePickAndPlace } from './simplePickAndPlace';

const TasksNodeParser = (exactType, dct) => {

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
        default:
            break;
    }

    return node;
};

export {
    CloseGripper,
    Initialize,
    OpenGripper,
    SimplePickAndPlace,
    TasksNodeParser
};