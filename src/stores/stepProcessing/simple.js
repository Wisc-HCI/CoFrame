import { DATA_TYPES } from "simple-vp/dist/components";
import { STATUS } from "../Constants";
import { findInstance, stepProcessors } from './index';
import { merge } from 'lodash';

export const simpleSteps = ({data, objectTypes, context, path, memo, solver, module, urdf}) => {
    console.log('simpleSteps: ',data.id)
    let status = STATUS.VALID;
    let updated = false;

    const objectType = objectTypes[data.type];

    let newMemo = {...memo};

    let newSteps = {[path]:[]};
    let lastTime = 0;
    
    Object.entries(objectType.properties).forEach(([property,propertyInfo])=>{
        if (propertyInfo.accepts && !propertyInfo.isList) {
            // Handles block-based properties

            // Retrieve the actual instance based on context
            const dataNode = findInstance(data.properties[property],context);
            const computeProps = {
                data:dataNode,
                objectTypes,
                context,
                path,
                memo: newMemo,
                solver,
                module,
                urdf
            }
            if (!dataNode && !propertyInfo.nullValid) {
                // If the node is null and it isn't allowed, log as a failure
                status = STATUS.FAILED;
            } else {
                // Recurse into the node
                const {
                    memo:innerMemo, 
                    status:innerStatus, 
                    updated:innerUpdated
                } = stepProcessors[objectTypes[dataNode.type].properties.computeSteps.default](computeProps)
                if (innerUpdated) {
                    // Child is updated, so the parent is as well.
                    updated = true;
                }
                if (innerStatus === STATUS.FAILED) {
                    // Status is failed, so the parent is also.
                    status = STATUS.FAILED;
                }
                // Update the memo to include the cached values
                newMemo = {...newMemo,...innerMemo};
            }
            
        } else if (propertyInfo.accepts && propertyInfo.isList) {
            // Handle the list-based properties. Generally these are children properties, 
            // and correspond to the steps of a hierarchical, program, or skill.
            data.properties[property].forEach(id=>{
                // Retrieve the actual instance based on context
                const dataNode = findInstance(id,context);
                const computeProps = {
                    data:dataNode,
                    objectTypes,
                    context,
                    path,
                    memo: newMemo,
                    solver,
                    module,
                    urdf
                }
                if (!dataNode && !propertyInfo.nullValid) {
                    // If the node is null and it isn't allowed, log as a failure
                    status = STATUS.FAILED;
                } else {
                    // Recurse into the node
                    const {
                        steps:innerSteps,
                        memo:innerMemo, 
                        status:innerStatus, 
                        updated:innerUpdated
                    } = stepProcessors[objectTypes[dataNode.type].properties.computeSteps.default](computeProps)
                    if (innerUpdated) {
                        // Child is updated, so the parent is as well.
                        updated = true;
                    }
                    if (innerStatus === STATUS.FAILED) {
                        // Status is failed, so the parent is also.
                        status = STATUS.FAILED;
                    }
                    // Update the memo to include the cached values
                    console.log(innerMemo);
                    const pathInnerSteps = innerSteps[path];
                    newMemo = {...newMemo,...innerMemo};
                    newSteps[path].push(pathInnerSteps.map(v=>({...v,time:v.time+lastTime})))
                    if (pathInnerSteps.length > 0) {
                        lastTime = pathInnerSteps[pathInnerSteps.length-1].time
                    }
                }

            })
        }
    })
    
    let dataMemo = {...data,properties:{...data.properties,status,steps:newSteps}};
    if (memo[data.id]) {
        dataMemo.properties.steps = merge(memo[data.id])
    }
    console.log(dataMemo)
    newMemo = {...newMemo,[data.id]:dataMemo};
    console.log(newMemo)

    return {steps:newSteps, memo:newMemo, status, updated}
}

/*
** LOGIC **

- search tree for nodes that are STATUS.PENDING
- If a node is STATUS.VALID, search internally and return the children so long as those children are also valid and are not updated.
- If a node is STATUS.FAILED or STATUS.PENDING, check memo. If there is an entry, return that, otherwise retry. Return result and updated = true, along with new status
- If a node is STATUS.VALID and has no children, return the previously calculated value from data.
- For a given node with children, status is STATUS.FAILED if any children failed, and STATUS.VALID otherwise
*/