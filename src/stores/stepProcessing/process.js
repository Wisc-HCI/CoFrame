import { STATUS, STEP_TYPE } from "../Constants";
import { findInstance } from './index';

export const processSteps = ({data, path, context, memo}) => {
    console.log('processSteps',data.id)
    let newSteps = {};
    let updated = false;
    let status = STATUS.VALID;
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
        const process = findInstance(data.properties.process,context);
        const machine = findInstance(data.properties.machine,context);
        const processMachine = findInstance(process?.properties.machine,context);

        // Define some statuses that are relevant
        const machineStartStatus = status === STATUS.VALID && machine ? {[machine.id]:{running:true}} : {};
        const processStartStatus = status === STATUS.VALID && process ? {[process.id]:{running:true},...machineStartStatus} : {};
        const machineFinishStatus = status === STATUS.VALID && machine ? {[machine.id]:{finish:true}} : {};
        const processFinishStatus = status === STATUS.VALID && process ? {[process.id]:{finish:true},...machineFinishStatus} : {};
        const machineStopStatus = status === STATUS.VALID && machine ? {[machine.id]:{stopped:true}} : {};
        const processStopStatus = status === STATUS.VALID && process ? {[process.id]:{stopped:true},...machineStopStatus} : {};

        let steps = [];
        status = machine === processMachine && process ? STATUS.VALID : STATUS.FAILED;
        console.log({process,machine,processMachine,status})
        if (data.type === 'processStartType') {
            steps = [
                {
                    stepType: STEP_TYPE.LANDMARK,
                    data: processStartStatus,
                    source: data.id,
                    time: 0
                },
                {
                    stepType: STEP_TYPE.LANDMARK,
                    data: processFinishStatus,
                    source: data.id,
                    time: process ? process.properties.processTime : 0
                }
            ]
        } else if (data.type === 'processWaitType') {
            steps = [
                {
                    stepType: STEP_TYPE.SCENE_UPDATE,
                    data: {},
                    source: data.id,
                    time: processFinishStatus
                }
            ];
        } else if (data.type === 'processStopType') {
            steps = [
                {
                    stepType: STEP_TYPE.LANDMARK,
                    data: processStopStatus,
                    source: data.id,
                    time: 0
                },
            ]
        }
        updated = true;
        newSteps = memo[data.id] ? {...memo[data.id].properties.steps,[path]:steps} : {[path]:steps};
    }
    // Pack the data/steps into the memo
    const dataMemo = {...data,properties:{...data.properties,status,steps:newSteps}};
    const newMemo = {...newMemo,[data.id]:dataMemo};
    return {steps:newSteps, memo:newMemo, status, updated}
}