import { STATUS, STEP_TYPE } from "../Constants";
import { findInstance, leafLogic } from './index';

export const processCompiler = ({data, path, context, memo}) => {
    return leafLogic({data,path,memo,context,updateFn:({data, path, context, memo})=>{
        const process = findInstance(data.properties.process,context);
        const machine = findInstance(data.properties.machine,context);
        const processMachine = findInstance(process?.properties.machine,context);

        // Define some statuses that are relevant
        // const machineStartStatus = status === STATUS.VALID && machine ? {[machine.id]:{running:true}} : {};
        // const processStartStatus = status === STATUS.VALID && process ? {[process.id]:{running:true},...machineStartStatus} : {};
        
        const machineStartStatus = status === STATUS.VALID && machine ? {[machine.id]:'started'} : {};
        const processStartStatus = status === STATUS.VALID && process ? {[process.id]:'started',...machineStartStatus} : {};
        const machineFinishStatus = status === STATUS.VALID && machine ? {[machine.id]:'finished'} : {};
        const processFinishStatus = status === STATUS.VALID && process ? {[process.id]:'finished',...machineFinishStatus} : {};
        const machineStopStatus = status === STATUS.VALID && machine ? {[machine.id]:'stopped'} : {};
        const processStopStatus = status === STATUS.VALID && process ? {[process.id]:'stopped',...machineStopStatus} : {};

        const startStepData = {
            machine: machine?machine.id:null,
            process: process?process.id:null,
            statuses: processStartStatus,
            id:data.id
        }

        const stopStepData = {
            machine: machine?machine.id:null,
            process: process?process.id:null,
            statuses: processStopStatus,
            id:data.id
        }

        let compiled = [];
        const status = machine === processMachine && process ? STATUS.VALID : STATUS.FAILED;

        if (data.type === 'processStartType') {
            compiled = [
                {
                    stepType: STEP_TYPE.PROCESS_START,
                    data: startStepData,
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
            compiled = [
                {
                    stepType: STEP_TYPE.ACTION_START,
                    data: {agent: 'robot',id:data.id},
                    source: data.id,
                    time: 0
                },
                {
                    stepType: STEP_TYPE.ACTION_END,
                    data: {agent: 'robot',id:data.id},
                    source: data.id,
                    time: processFinishStatus
                },
            ];
        } else if (data.type === 'processStopType') {
            compiled = [
                {
                    stepType: STEP_TYPE.PROCESS_END,
                    data: stopStepData,
                    source: data.id,
                    time: 0
                }
            ]
        }

        return {compiled,status}
    }})
}