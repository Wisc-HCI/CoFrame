import { Delay } from './delay';
import { Gripper } from './gripper';
import { MoveTrajectory } from './move_trajectory';
import { MoveUnplanned } from './move_unplanned';


const PrimitiveNodeParser = (exactType, dct) => {

    let node = null;

    switch(exactType) {
        case 'move-trajectory':
            node = MoveTrajectory.fromDict(dct);
            break;
        case 'move-unplanned':
            node = MoveUnplanned.fromDict(dct);
            break;
        case 'delay':
            node = Delay.fromDict(dct);
            break;
        case 'gripper':
            node = Gripper.fromDict(dct);
            break;
    }

    return node;
};

export {
    Delay,
    Gripper,
    MoveTrajectory,
    MoveUnplanned,
    PrimitiveNodeParser
};