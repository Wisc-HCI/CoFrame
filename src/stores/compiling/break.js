import { STATUS, STEP_TYPE } from "../Constants";

export const breakCompiler = ({ data }) => {
    const newCompiled = {
        break: true,
        steps: [
            {
                stepType: STEP_TYPE.LANDMARK,
                source: data.id,
                time: 0
            }
        ]
    }
    return { newCompiled, status: STATUS.VALID, break: true }
}

// - If a node is STATUS.VALID, search internally and return the children so long as those children are also valid and are not updated.
// - If a node is STATUS.FAILED or STATUS.PENDING, check memo. If there is an entry, return that, otherwise retry. Return result and updated = true, along with new status
// - If a node is STATUS.VALID and has no children, return the previously calculated value from data.