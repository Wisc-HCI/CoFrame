import { STATUS, STEP_TYPE } from "../Constants";

export const breakCompiler = ({ data }) => {
    const newCompiled = {
        shouldBreak: true,
        status: STATUS.VALID,
        steps: [
            {
                stepType: STEP_TYPE.LANDMARK,
                data: {label: 'Program Break'},
                source: data.id,
                time: 0
            }
        ]
    }
    return newCompiled
}