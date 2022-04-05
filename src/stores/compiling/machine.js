import { STATUS, STEP_TYPE } from "../Constants";

export const machineCompiler = ({data, properties}) => {
    const machine = properties.machine;
    
    const status = machine.id ? STATUS.VALID : STATUS.FAILED;

    const newCompiled = machine ? {
        status,
        shouldBreak: false,
        steps: [
            {
                stepType: STEP_TYPE.LANDMARK,
                data: {machine: machine.id,label: 'Machine Initialized'},
                source: data.id,
                time: 0
            }
        ]
    } : {
        status,
        shouldBreak: false,
        steps:[]
    }
    return newCompiled
}