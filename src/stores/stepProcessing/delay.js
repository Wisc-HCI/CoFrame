import { STATUS, STEP_TYPE } from "../Constants";

export const delaySteps = ({data, path, memo}) => {
    const step = {
        stepType: STEP_TYPE.SCENE_UPDATE,
        data: {},
        time: data.properties.duration
    }
    const newSteps = memo[data.id] ? {...memo[data.id].properties.steps,[path]:[step]} : {[path]:[step]};
    const dataMemo = {...data,properties:{...data.properties,status:STATUS.VALID,steps:newSteps}};
    const newMemo = {...newMemo,[data.id]:dataMemo};
    return {steps:newSteps, memo:newMemo, status:STATUS.VALID, updated:false}
}