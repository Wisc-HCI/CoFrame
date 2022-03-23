import { STATUS, STEP_TYPE } from "../Constants";
import { leafLogic } from './index';

export const gripperMotionSteps = ({data, path, memo}) => {
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