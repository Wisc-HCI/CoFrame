import { STATUS, STEP_TYPE } from "../Constants";

export const delaySteps = ({data, path, memo}) => {
    let newSteps = {};
    let updated = false;
    if (data.properties.status === STATUS.VALID && (data.properties.steps[path] || memo[data.id].properties.steps[path])) {
        // Use the version on file
        const prevPath = data.properties.steps[path]
        newSteps = memo[data.id] === undefined
            ? {[path]:prevPath}
            : memo[data.id].properties.steps[path] 
            ? memo[data.id].properties.steps
            : {...memo[data.id].properties.steps,[path]:prevPath}
    } else {
        // No cached version, or needs update
        const step = {
            stepType: STEP_TYPE.SCENE_UPDATE,
            data: {},
            source: data.id,
            time: data.properties.duration
        }
        updated = true;
        newSteps = memo[data.id] ? {...memo[data.id].properties.steps,[path]:[step]} : {[path]:[step]};
    }
    // Pack the data/steps into the memo
    const dataMemo = {...data,properties:{...data.properties,status:STATUS.VALID,steps:newSteps}};
    const newMemo = {...newMemo,[data.id]:dataMemo};
    return {steps:newSteps, memo:newMemo, status:STATUS.VALID, updated}
}

// - If a node is STATUS.VALID, search internally and return the children so long as those children are also valid and are not updated.
// - If a node is STATUS.FAILED or STATUS.PENDING, check memo. If there is an entry, return that, otherwise retry. Return result and updated = true, along with new status
// - If a node is STATUS.VALID and has no children, return the previously calculated value from data.