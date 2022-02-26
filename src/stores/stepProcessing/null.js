import { STATUS } from "../Constants";

export const nullSteps = ({}) => {
    return {steps:[], memo:{}, status:STATUS.VALID, updated:false}
}