import { DATA_TYPES } from "simple-vp";
import { STATUS, STEP_TYPE } from "../Constants";
import { findInstance, leafLogic } from './index';

export const agentSteps = ({data, path, context, memo}) => {
    console.log('agentSteps')
    return leafLogic({data,path,memo,context,updateFn:({data, context})=>{
        // For now, always assume the robot agent is initialized
        const roboAgents = Object.values(context).filter(item=>item.type==='robotAgentType'&&item.dataType===DATA_TYPES.INSTANCE);
        const agent = roboAgents.length > 0 ? roboAgents[0] : null;
        const location = findInstance(data.properties.initialPosition,context);
        const gripState = data.properties.initialGripper;
        const valid = location && agent && gripState <= 85 && gripState >= 0;
        const status = valid ? STATUS.VALID : STATUS.FAILED;
        const steps = valid ? [{
            stepType: STEP_TYPE.LANDMARK,
            data: {[agent.id]:{initialized:true,code:'agentInitialized'}},
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