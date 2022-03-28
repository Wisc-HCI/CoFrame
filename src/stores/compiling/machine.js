import { STATUS, STEP_TYPE } from "../Constants";

export const machineCompiler = ({data, properties}) => {
    const machine = properties.machine;
    
    const status = machine ? STATUS.VALID : STATUS.FAILED;

    const newCompiled = machine ? {
        steps: [
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
                time: properties.duration
            }
        ]
    } : null
    return { newCompiled, status }
}