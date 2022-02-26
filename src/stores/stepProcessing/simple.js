import { STATUS } from "../Constants";
import { stepProcessors } from './index';

export const simpleSteps = ({data, objectTypes, context, memo, solver, module, urdf}) => {

    let status = STATUS.VALID;
    let updated = false;

    const objectType = objectTypes[data.type];
    
    Object.entries(objectType.properties).forEach(([property,propertyInfo])=>{
        if (propertyInfo.accepts && property !== 'children') {
            const {innerStatus:status} = {};
        } else {

        }
    })
    


    return {steps:[], memo:{}, status, updated}
}

/*
** LOGIC **

- search tree for nodes that are STATUS.PENDING
- If a node is STATUS.VALID, search internally and return the children so long as those children are also valid and are not updated.
- If a node is STATUS.FAILED or STATUS.PENDING, check memo. If there is an entry, return that, otherwise retry. Return result and updated = true, along with new status
- If a node is STATUS.VALID and has no children, return the previously calculated value from data.
- For a given node with children, status is STATUS.FAILED if any children failed, and STATUS.VALID otherwise
*/