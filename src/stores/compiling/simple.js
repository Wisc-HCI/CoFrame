import { STATUS, STEP_TYPE } from "../Constants";
import { findInstance, compilers } from './index';
import { merge } from 'lodash';
import { equals } from "./index";

export const simpleCompiler = ({data, objectTypes, context, path, memo, solver, module, urdf, worldModel}) => {
    // console.log('simpleSteps',data.id)
    // console.log(objectTypes)
    let status = STATUS.VALID;
    let updated = false;

    const objectType = objectTypes[data.type];

    let newMemo = {...memo};

    let newCompiled = {[path]:[]};
    let lastTime = 0;
    let shouldBreak = false;
    
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
                urdf,
                worldModel
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
                } = compilers[objectTypes[dataNode.type].properties.compileFn.default](computeProps)
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
            // and correspond to the compiled of a hierarchical, program, or skill.
            data.properties[property].some(id=>{
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
                    urdf,
                    worldModel
                }
                if (!dataNode && !propertyInfo.nullValid) {
                    // If the node is null and it isn't allowed, log as a failure
                    status = STATUS.FAILED;
                    return false
                } else {
                    // Recurse into the node
                    // console.log(objectTypes[dataNode.type].properties.compileFn)
                    // console.log(compilers[objectTypes[dataNode.type].properties.compileFn.default])
                    const {
                        compiled:innerCompiled,
                        memo:innerMemo, 
                        status:innerStatus, 
                        updated:innerUpdated,
                        break:innerBreak
                    } = compilers[objectTypes[dataNode.type].properties.compileFn.default](computeProps)
                    if (innerUpdated) {
                        // Child is updated, so the parent is as well.
                        updated = true;
                    }
                    if (innerStatus === STATUS.FAILED) {
                        // Status is failed, so the parent is also.
                        status = STATUS.FAILED;
                    }
                    // Update the memo to include the cached values
                    const pathInnerSteps = innerCompiled[path];
                    newMemo = merge(newMemo,innerMemo);
                    let lastUpdateTime = 0;
                    pathInnerSteps.forEach(v=>{
                        let stepTime = 0;
                        if (typeof v.time === "object") {
                            // encode the timing based on the landmark specified
                            const tempSteps = newCompiled[path].slice().reverse();
                            tempSteps.some(s=>{
                                if (s.stepType === STEP_TYPE.LANDMARK && equals(s.data,v.time)) {
                                    stepTime = s.time
                                    return false
                                } else if (s.time < lastUpdateTime) {
                                    return true
                                } else {
                                    return false
                                }
                            })
                        } else {
                            stepTime = v.time
                        }
                        if (v.stepType !== STEP_TYPE.LANDMARK) {
                            // Ignore landmarks
                            lastUpdateTime = stepTime
                        }
                        newCompiled[path].push({...v,time:stepTime+lastTime})
                    })
                    if (pathInnerSteps.length > 0) {
                        lastTime = lastTime + lastUpdateTime
                    }
                    if (innerBreak) {
                        shouldBreak = true;
                        return true
                    }
                    return false
                }

            })
        }
    })
    newCompiled[path].sort((stepA,stepB)=>stepA.time-stepB.time);
    let dataMemo = {...data,properties:{...data.properties,status,compiled:newCompiled}};
    if (memo[data.id]) {
        dataMemo.properties.compiled = merge(memo[data.id])
    }
    newMemo = {...newMemo,[data.id]:dataMemo};

    return {compiled:newCompiled, memo:newMemo, status, updated, break:shouldBreak}
}

/*
** LOGIC **

- search tree for nodes that are STATUS.PENDING
- If a node is STATUS.VALID, search internally and return the children so long as those children are also valid and are not updated.
- If a node is STATUS.FAILED or STATUS.PENDING, check memo. If there is an entry, return that, otherwise retry. Return result and updated = true, along with new status
- If a node is STATUS.VALID and has no children, return the previously calculated value from data.
- For a given node with children, status is STATUS.FAILED if any children failed, and STATUS.VALID otherwise
*/