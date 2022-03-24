import { STATUS, STEP_TYPE } from "../Constants";
import { leafLogic } from './index';

export const gripperMotionCompiler = ({data, path, memo}) => {
    return leafLogic({data,path,memo,updateFn:({data})=>{
        const compiled = [{
            stepType: STEP_TYPE.LANDMARK,
            data: {},
            source: data.id,
            time: 0
        }]
        return {compiled,status:STATUS.VALID,break:true}
    }})
}