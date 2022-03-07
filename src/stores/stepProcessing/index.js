import { delaySteps } from "./delay";
import { gripperMotionSteps } from "./gripperMotion";
import { machineSteps } from "./machine";
import { nullSteps } from './null';
import { poseSteps } from './pose';
import { processSteps } from './process';
import { robotMotionSteps } from './robotMotion';
import { simpleSteps } from './simple';
import { skillSteps } from './skill';

import { STEP_CALCULATOR, STATUS } from "../Constants";
import { DATA_TYPES } from "simple-vp";

const KEY_MAPPING = {
    NULL: nullSteps,
    SIMPLE: simpleSteps,
    MACHINE: machineSteps,
    DELAY: delaySteps,
    PROCESS: processSteps,
    SKILL: skillSteps,
    POSE: poseSteps,
    GRIPPER: gripperMotionSteps,
    ROBOT_MOTION: robotMotionSteps
}
// Ordering corresponds to the values in the STEP_CALCULATOR constant

export const stepProcessors =  Object.keys(STEP_CALCULATOR).map(k=>KEY_MAPPING[k]);

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

export const leafLogic = ({data, objectTypes, context, path, memo, solver, module, urdf, updateFn}) => {
    let newSteps = {};
    let updated = false;
    let status = STATUS.VALID;
    if (memo[data.id]?.properties.steps[path]) {
        // Use the version in the memo
        console.log('using memoized version of ',data.id)
        newSteps = memo[data.id].properties.steps;
        status = memo[data.id].properties.status;
        // Record a change if the status was previously pending
        updated = data.properties.status === STATUS.PENDING;
    } else if ([STATUS.FAILED,STATUS.VALID].includes(data.properties.status) && data.properties.steps[path]) {
        // Use the version calculated previously
        console.log('using cached version of ',data.id)
        const prevPath = data.properties.steps[path]
        newSteps = memo[data.id] === undefined
            ? {[path]:prevPath}
            : memo[data.id].properties.steps[path] 
            ? memo[data.id].properties.steps
            : {...memo[data.id].properties.steps,[path]:prevPath}
        // Record a change if the status was previously pending
        status = data.properties.status;
        updated = data.properties.status === STATUS.PENDING;
    } else {
        console.log('updating version of ',data.id)
        let result = updateFn({data, objectTypes, context, path, memo, solver, module, urdf, updateFn});
        const steps = result.steps;
        updated = true;
        status = result.status;
        console.log(result.status)
        newSteps = memo[data.id] ? {...memo[data.id].properties.steps,[path]:steps} : {[path]:steps};
    }
    // Pack the data/steps into the memo
    const dataMemo = {...data,properties:{...data.properties,status,steps:newSteps}};
    const newMemo = {...newMemo,[data.id]:dataMemo};
    console.log(newMemo)
    return {steps:newSteps, memo:newMemo, status, updated}
}