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
import { agentCompiler } from "./agent";

import { COMPILE_FUNCTIONS, STATUS } from "../Constants";
import { DATA_TYPES } from "simple-vp";

const KEY_MAPPING = {
    NULL: nullCompiler,
    SIMPLE: simpleCompiler,
    MACHINE: machineCompiler,
    DELAY: delayCompiler,
    PROCESS: processCompiler,
    SKILL: skillCompiler,
    POSE: poseCompiler,
    GRIPPER: gripperMotionCompiler,
    ROBOT_MOTION: robotMotionCompiler,
    BREAK: breakCompiler,
    AGENT: agentCompiler
}
// Ordering corresponds to the values in the COMPILE_FUNCTIONS constant

export const compilers =  Object.keys(COMPILE_FUNCTIONS).map(k=>KEY_MAPPING[k]);

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
    let newCompiled = {};
    let updated = false;
    let status = STATUS.VALID;
    let shouldBreak = false;
    if (memo[data.id]?.properties.compiled[path]) {
        // Use the version in the memo
        console.log('using memoized version of ',data.id)
        newCompiled = memo[data.id].properties.compiled;
        status = memo[data.id].properties.status;
        newCompiled[path].some(step=>{
            if (step.data.break) {
                shouldBreak = true
                return true
            } else {
                return false
            }
        })
        // Record a change if the status was previously pending
        updated = data.properties.status === STATUS.PENDING;
    } else if ([STATUS.FAILED,STATUS.VALID].includes(data.properties.status) && data.properties.compiled[path]) {
        // Use the version calculated previously
        console.log('using cached version of ',data.id)
        const prevPath = data.properties.compiled[path]
        newCompiled = memo[data.id] === undefined
            ? {[path]:prevPath}
            : memo[data.id].properties.compiled[path] 
            ? memo[data.id].properties.compiled
            : {...memo[data.id].properties.compiled,[path]:prevPath}
        newCompiled[path].some(step=>{
            if (step.data.break) {
                shouldBreak = true
                return true
            } else {
                return false
            }
        })
        // Record a change if the status was previously pending
        status = data.properties.status;
        updated = data.properties.status === STATUS.PENDING;
    } else {
        console.log('updating version of ',data.id)
        let result = updateFn({data, objectTypes, context, path, memo, solver, module, urdf, updateFn});
        const compiled = result.compiled;
        updated = true;
        status = result.status;
        shouldBreak = result.break;
        newCompiled = memo[data.id] ? {...memo[data.id].properties.compiled,[path]:compiled} : {[path]:compiled};
    }
    // Pack the data/compiled into the memo
    const dataMemo = {...data,properties:{...data.properties,status,compiled:newCompiled}};
    const newMemo = {...newMemo,[data.id]:dataMemo};
    console.log(newMemo)
    return {compiled:newCompiled, memo:newMemo, status, updated, break:shouldBreak}
}