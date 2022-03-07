import { STATUS, STEP_TYPE } from "../Constants";
import { findInstance, leafLogic } from './index';

export const processSteps = ({data, path, context, memo}) => {
    return leafLogic({data,path,memo,context,updateFn:({data, path, context, memo})=>{
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
        const status = machine === processMachine && process ? STATUS.VALID : STATUS.FAILED;

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

        return {steps,status}
    }})
}