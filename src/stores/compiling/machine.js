import { STATUS, STEP_TYPE } from "../Constants";
import { findInstance, leafLogic } from './index';

export const machineCompiler = ({data, path, context, memo}) => {
    console.log('machineCompiler')
    return leafLogic({data,path,memo,context,updateFn:({data, context})=>{
        console.log('update machineCompiler')
        const machine = findInstance(data.properties.machine,context);
        const status = machine ? STATUS.VALID : STATUS.FAILED;
        console.log({status})
        const compiled = machine ? [{
            stepType: STEP_TYPE.LANDMARK,
            data: {[machine.id]:{initialized:true,code:'machineInitialized'}},
            source: data.id,
            time: 0
        }] : [{
            stepType: STEP_TYPE.LANDMARK,
            data: {},
            source: data.id,
            time: 0
        }]
        return {compiled,status}
    }})
}