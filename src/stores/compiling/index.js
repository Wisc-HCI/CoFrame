import { delayCompiler } from "./delay";
import { gripperMotionCompiler } from "./gripperMotion";
import { machineCompiler } from "./machine";
import { nullCompiler } from './null';
import { poseCompiler } from './pose';
import { processCompiler } from './process';
import { robotMotionCompiler } from './robotMotion';
import { simpleCompiler } from './simple';
import { skillCompiler } from './skill';
import { breakCompiler } from "./break";
import { robotAgentCompiler } from "./robotAgent";
import { humanAgentCompiler } from "./humanAgent";
import { linkCompiler } from "./link";
import { propertyCompiler } from "./property";
import lodash from 'lodash';
import { COMPILE_FUNCTIONS, STATUS } from "../Constants";
import { DATA_TYPES } from "simple-vp";
import { gripperCompiler } from "./gripper";

const KEY_MAPPING = {
    NULL: nullCompiler,
    SIMPLE: simpleCompiler,
    MACHINE: machineCompiler,
    DELAY: delayCompiler,
    BREAK: breakCompiler,
    PROCESS: processCompiler,
    SKILL: skillCompiler,
    POSE: poseCompiler,
    ROBOT_MOTION: robotMotionCompiler,
    GRIPPER_MOTION: gripperMotionCompiler,
    ROBOT_AGENT: robotAgentCompiler,
    HUMAN_AGENT: humanAgentCompiler,
    GRIPPER: gripperCompiler,
    LINK: linkCompiler,
    PROPERTY: propertyCompiler

}
// Ordering corresponds to the values in the COMPILE_FUNCTIONS constant

export const compilers = Object.keys(COMPILE_FUNCTIONS).map(k => KEY_MAPPING[k]);

export const findInstance = (id, context) => {
    let found = false;
    let stale = false;
    let data = undefined;
    let dataId = id;

    while (!found && !stale) {
        data = context[dataId];
        if (!data) {
            stale = true
        } else if (data.dataType === DATA_TYPES.INSTANCE) {
            found = true;
        } else if (data.dataType === DATA_TYPES.REFERENCE) {
            dataId = data.ref;
        } else {
            stale = true
        }
    }

    return data;
}

export const equals = (a, b) => JSON.stringify(a) === JSON.stringify(b);

// export const leafLogic = ({ data, objectTypes, context, path, memo, module, updateFn }) => {
//     let newCompiled = {};
//     let updated = false;
//     let status = STATUS.VALID;
//     let shouldBreak = false;
//     if (memo[data.id]?.properties.compiled[path]) {
//         // Use the version in the memo
//         console.log('using memoized version of ', data.id)
//         newCompiled = memo[data.id].properties.compiled;
//         status = memo[data.id].properties.status;
//         newCompiled[path].some(step => {
//             if (step.data.break) {
//                 shouldBreak = true
//                 return true
//             } else {
//                 return false
//             }
//         })
//         // Record a change if the status was previously pending
//         updated = data.properties.status === STATUS.PENDING;
//     } else if ([STATUS.FAILED, STATUS.VALID].includes(data.properties.status) && data.properties.compiled[path]) {
//         // Use the version calculated previously
//         console.log('using cached version of ', data.id)
//         const prevPath = data.properties.compiled[path]
//         newCompiled = memo[data.id] === undefined
//             ? { [path]: prevPath }
//             : memo[data.id].properties.compiled[path]
//                 ? memo[data.id].properties.compiled
//                 : { ...memo[data.id].properties.compiled, [path]: prevPath }
//         newCompiled[path].some(step => {
//             if (step.data.break) {
//                 shouldBreak = true
//                 return true
//             } else {
//                 return false
//             }
//         })
//         // Record a change if the status was previously pending
//         status = data.properties.status;
//         updated = data.properties.status === STATUS.PENDING;
//     } else {
//         console.log('updating version of ', data.id)
//         let result = updateFn({ data, objectTypes, context, path, memo, solver, module, urdf, updateFn });
//         const compiled = result.compiled;
//         updated = true;
//         status = result.status;
//         shouldBreak = result.break;
//         newCompiled = memo[data.id] ? { ...memo[data.id].properties.compiled, [path]: compiled } : { [path]: compiled };
//     }
//     // Pack the data/compiled into the memo
//     const dataMemo = { ...data, properties: { ...data.properties, status, compiled: newCompiled } };
//     const newMemo = { ...newMemo, [data.id]: dataMemo };
//     console.log(newMemo)
//     return { compiled: newCompiled, memo: newMemo, status, updated, break: shouldBreak }
// }

// const shouldBreak = (nodeData) => {
//     let shouldBreak = false
//     nodeData.properties.compiled[path].steps?.some(step => {
//         if (step.data.break) {
//             shouldBreak = true
//             return true
//         } else {
//             return false
//         }
//     })
//     return shouldBreak
// }

// Copies data from a memo into the specified node.
// Returns standard changes, which will need to be propagated back up.
const copyMemoizedData = (memoizedData, data, path) => {
    // console.warn('copying memoized version of ', data)
    return [
        memoizedData, // memoizedData
        memoizedData.properties.compiled[path].status, // status
        data.properties.status === STATUS.PENDING, // updated
        memoizedData.properties.compiled[path].break // shouldBreak
    ]
}

// Copies data into memoized data.
// Returns standard changes, which will need to be propagated back up.
const updateMemoizedData = (memoizedData, data, path) => {
    // console.warn('updating memoized version of ', data);
    let newMemoizedData = lodash.merge({ properties: { compiled: { [path]: {} } } }, memoizedData);
    const pastCompiled = data.properties.compiled[path];
    newMemoizedData.properties.compiled[path] = lodash.merge(newMemoizedData.properties.compiled[path], pastCompiled)
    if (newMemoizedData.properties.compiled[path].otherPropertyUpdates) {
        // console.log('Has other properties, updating...')
        newMemoizedData = lodash.merge(newMemoizedData, { properties: newMemoizedData.properties.compiled[path].otherPropertyUpdates });
        // console.log('New with updates:', newMemoizedData)
    }
    return [
        newMemoizedData, // memoizedData
        pastCompiled.status, // status
        data.properties.status === STATUS.PENDING, // updated
        pastCompiled.break // shouldBreak
    ]
}

// Computes a new node, given that children have been updated.
// Returns standard changes, which will need to be propagated back up.
const performUpdate = (memoizedData, data, properties, objectTypes, context, path, memo, module, worldModel, updateFn) => {
    // console.warn('updating version of ', data);
    let newCompiled = updateFn({ data, properties, objectTypes, context, path, memo, module, worldModel });
    // console.log('status',newCompiled.status)
    // console.log('memoizedData',memoizedData)
    // console.log('newCompiled',newCompiled)
    let newMemoizedData = lodash.merge({ properties: { compiled: { [path]: {} } } }, memoizedData);
    newMemoizedData.properties.compiled[path] = lodash.merge(newMemoizedData.properties.compiled[path], newCompiled)
    if (newCompiled.otherPropertyUpdates) {
        // console.log('Has other properties, updating...')
        newMemoizedData = lodash.merge(newMemoizedData, { properties: newCompiled.otherPropertyUpdates });
        // console.log('New with updates:', newMemoizedData)
    }
    // console.log(newMemoizedData)
    return [
        newMemoizedData, // memoizedData
        newCompiled.status, // status
        data.properties.status === STATUS.PENDING, // updated
        newCompiled.shouldBreak // shouldBreak
    ]
}

// Computes a property of a parent given the field data and field info.
// Can handle node-based and 'ignored' fields
// Returns standard changes, which will need to be propagated back up.
const computeProperty = (fieldValue, fieldInfo, objectTypes, context, path, memo, module, worldModel) => {
    let memoizedData = {};
    let newMemo = { ...memo };
    let status = STATUS.VALID;
    let updated = false;
    let shouldBreak = false

    // First, if the field value is null, return null, with checkign for validity
    if (!fieldValue && !fieldInfo.nullValid) {
        status = STATUS.FAILED
    }
    if (!fieldValue) {
        return { memoizedData, memo: newMemo, status, updated, shouldBreak }
    }

    if (fieldInfo.accepts) {
        const propData = findInstance(fieldValue, context);
        // Check cases where the result is null here
        if (!propData && fieldInfo.nullValid) {
            return { memoizedData, memo: newMemo, status, updated, shouldBreak }
        } else if (!propData) {
            status = STATUS.FAILED;
            return { memoizedData, memo: newMemo, status, updated, shouldBreak }
        }
        // Since the prop is valid, continue
        const computeProps = {
            data: propData,
            objectTypes,
            context,
            path,
            memo: newMemo,
            module,
            worldModel
        }

        const {
            memoizedData: innerMemoizedData,
            memo: innerMemo,
            status: innerStatus,
            updated: innerUpdated,
            shouldBreak: innerShouldBreak
        } = handleUpdate(computeProps);
        if (innerUpdated) {
            updated = true
        };
        if (innerStatus === STATUS.FAILED) {
            // Status is failed, so the parent is also.
            status = STATUS.FAILED;
        }
        if (innerShouldBreak) {
            shouldBreak = true;
        }
        memoizedData = innerMemoizedData;
        newMemo = lodash.merge(newMemo, innerMemo);
    } else {
        memoizedData = fieldValue;
    }

    return { memoizedData, memo: newMemo, status, updated, shouldBreak }
}

// Entry for recursive process of updating data. 
// Returns standard changes, which will need to be propagated back up.
export const handleUpdate = ({ data, objectTypes, context, path, memo, module, worldModel }) => {
    let newMemo = { ...memo };
    // console.warn('DATA:',data.id)
    const updateFn = compilers[objectTypes[data.type].properties.compileFn.default];
    let memoizedData = memo[data.id] ? memo[data.id] : {};
    let updated = false;
    let status = STATUS.VALID;
    let shouldBreak = false;
    if (memoizedData.properties?.compiled[path]) {
        // Use the version in the memo
        [memoizedData, status, updated, shouldBreak] = copyMemoizedData(memoizedData, data, path);
    } else {
        // First, determine whether we need to compute. 
        // If the status is pending, recompute.
        // If not pending, check the `updateFields` for node-based updates.
        // If any of those have updates, recompute.
        let recompute = data.properties.status === STATUS.PENDING;

        // No change detected at this level, but we should progress though `updateFields` to be sure.
        let properties = {};
        data.properties.updateFields.forEach(field => {
            // console.log('checking field', field)
            if (objectTypes[data.type].properties[field].isList) {
                properties[field] = [];
                data.properties[field].some(fieldItem => {
                    const {
                        memoizedData: innerMemoizedData,
                        memo: innerMemo,
                        status: innerStatus,
                        updated: innerUpdated,
                        shouldBreak: innerShouldBreak
                    } = computeProperty(
                        fieldItem, objectTypes[data.type].properties[field],
                        objectTypes, context, path, memo, module, worldModel
                    )
                    properties[field].push(innerMemoizedData);
                    newMemo = lodash.merge(newMemo, innerMemo);
                    if (innerUpdated) {
                        updated = true;
                    };
                    if (innerStatus === STATUS.FAILED) {
                        // Status is failed, so the parent is also.
                        status = STATUS.FAILED;
                    }
                    if (innerShouldBreak) {
                        shouldBreak = true;
                    }
                    return innerShouldBreak
                })
            } else {
                const {
                    memoizedData: innerMemoizedData,
                    memo: innerMemo,
                    status: innerStatus,
                    updated: innerUpdated,
                    shouldBreak: innerShouldBreak
                } = computeProperty(
                    data.properties[field], objectTypes[data.type].properties[field],
                    objectTypes, context, path, memo, module, worldModel
                )
                properties[field] = innerMemoizedData;
                newMemo = lodash.merge(newMemo, innerMemo);
                if (innerUpdated) {
                    updated = true
                };
                if (innerStatus === STATUS.FAILED) {
                    // Status is failed, so the parent is also.
                    status = STATUS.FAILED;
                };
                if (innerShouldBreak) {
                    shouldBreak = true;
                }
            }
        })

        if (recompute || updated) {
            [memoizedData, status, updated, shouldBreak] = performUpdate(
                memoizedData, data, properties, objectTypes, context, path,
                newMemo, module, worldModel, updateFn
            );
            updated = true;
        } else {
            [memoizedData, status, updated, shouldBreak] = updateMemoizedData(memoizedData, data, path);
        }
    }

    // console.warn(`At the end of processing "${data.id}", status is "${status === STATUS.VALID ? 'VALID' : status === STATUS.PENDING ? 'PENDING' : 'FAILED'}"`)
    if (status === STATUS.VALID && (memoizedData.properties.status === undefined || memoizedData.properties.status !== STATUS.FAILED)) {
        // console.warn('SETTING STATUS VALID')
        memoizedData.properties.status = STATUS.VALID
    } else if (status === STATUS.FAILED) {
        // console.warn('SETTING STATUS FAILED')
        memoizedData.properties.status = STATUS.FAILED
    }
    memoizedData.properties.compiled[path].break = shouldBreak
    memoizedData.type = data.type;
    memoizedData.id = data.id;
    // console.log(memoizedData)

    newMemo = { ...newMemo, [data.id]: memoizedData };
    return { memoizedData, memo: newMemo, status, updated, shouldBreak }
}