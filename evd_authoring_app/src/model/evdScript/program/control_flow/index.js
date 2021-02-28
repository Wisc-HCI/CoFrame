import { Branch } from './branch';
import { Breakpoint } from './breakpoint';
import { Loop } from './loop';


const ControlFlowNodeParser = (exactType, dct) => {

    let node = null;

    switch(exactType) {
        case 'loop':
            node = Loop.fromDict(dct);
            break;
        case 'branch':
            node = Branch.fromDict(dct);
            break;
        case 'breakpoint':
            node = Breakpoint.fromDict(dct);
            break;
        default:
            break;
    }

    return node;
};

export {
    Branch,
    Breakpoint,
    Loop,
    ControlFlowNodeParser
};