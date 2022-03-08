import { STATUS, STEP_TYPE } from "../Constants";
import { leafLogic } from './index';

export const breakSteps = ({data, path, memo}) => {
    return leafLogic({data,path,memo,updateFn:({data})=>{
        const steps = [{
            stepType: STEP_TYPE.LANDMARK,
            data: {break:true},
            source: data.id,
            time: 0
        }]
        return {steps,status:STATUS.VALID,break:true}
    }})
}

// - If a node is STATUS.VALID, search internally and return the children so long as those children are also valid and are not updated.
// - If a node is STATUS.FAILED or STATUS.PENDING, check memo. If there is an entry, return that, otherwise retry. Return result and updated = true, along with new status
// - If a node is STATUS.VALID and has no children, return the previously calculated value from data.