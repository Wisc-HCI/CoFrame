import { STATUS, STEP_TYPE } from "../Constants";

export const gripperMotionCompiler = ({data, path, memo}) => {
    const newCompiled = {
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
                time: data.properties.duration
            }
        ]
    }
    return { newCompiled, status: STATUS.VALID }
}