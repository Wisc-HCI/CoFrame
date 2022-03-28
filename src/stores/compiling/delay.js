import { STATUS, STEP_TYPE } from "../Constants";

export const delayCompiler = ({data}) => {
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

// - If a node is STATUS.VALID, search internally and return the children so long as those children are also valid and are not updated.
// - If a node is STATUS.FAILED or STATUS.PENDING, check memo. If there is an entry, return that, otherwise retry. Return result and updated = true, along with new status
// - If a node is STATUS.VALID and has no children, return the previously calculated value from data.