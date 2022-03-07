import { STATUS, STEP_TYPE } from "../Constants";
import { findInstance, leafLogic } from './index';

export const machineSteps = ({data, path, context, memo}) => {
    console.log('machineSteps')
    return leafLogic({data,path,memo,context,updateFn:({data, context})=>{
        console.log('update machineSteps')
        const machine = findInstance(data.properties.machine,context);
        const status = machine ? STATUS.VALID : STATUS.FAILED;
        console.log({status})
        const steps = machine ? [{
            stepType: STEP_TYPE.LANDMARK,
            data: {[machine.id]:{initialized:true}},
            source: data.id,
            time: 0
        }] : [{
            stepType: STEP_TYPE.LANDMARK,
            data: {},
            source: data.id,
            time: 0
        }]
        return {steps,status}
    }})
}