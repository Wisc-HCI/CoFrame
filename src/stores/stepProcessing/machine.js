import { STATUS, STEP_TYPE } from "../Constants";
import { findInstance } from './index';

export const machineSteps = ({data, path, context, memo}) => {
    console.log('machineSteps',data.id)
    let newSteps = {};
    let updated = false;
    let status = STATUS.VALID
    if (memo[data.id]?.properties.steps[path] && memo[data.id]?.properties.status === STATUS.VALID) {
        // Use the version in the memo
        newSteps = memo[data.id].properties.steps
        // Record a change if the status was previously pending
        updated = data.properties.status === STATUS.PENDING;
    } else if (data.properties.status === STATUS.VALID && data.properties.steps[path]) {
        // Use the version calculated previously
        const prevPath = data.properties.steps[path]
        newSteps = memo[data.id] === undefined
            ? {[path]:prevPath}
            : memo[data.id].properties.steps[path] 
            ? memo[data.id].properties.steps
            : {...memo[data.id].properties.steps,[path]:prevPath}
        // Record a change if the status was previously pending
        updated = data.properties.status === STATUS.PENDING;
    } else {
        // No cached version, or needs update
        const machine = findInstance(data.properties.machine,context);
        console.log('machine',machine)
        status = machine ? STATUS.VALID : STATUS.FAILED;
        const step = machine ? {
            stepType: STEP_TYPE.LANDMARK,
            data: {[machine.id]:{initialized:true}},
            time: 0
        } : {
            stepType: STEP_TYPE.LANDMARK,
            data: {},
            time: 0
        }
        updated = true;
        newSteps = memo[data.id] ? {...memo[data.id].properties.steps,[path]:[step]} : {[path]:[step]};
    }
    // Pack the data/steps into the memo
    const dataMemo = {...data,properties:{...data.properties,status,steps:newSteps}};
    const newMemo = {...newMemo,[data.id]:dataMemo};
    return {steps:newSteps, memo:newMemo, status, updated}
}