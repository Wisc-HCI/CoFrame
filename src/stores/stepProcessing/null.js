import { STATUS } from "../Constants";

export const nullSteps = ({path}) => {
    return {steps:{[path]:[]}, memo:{}, status:STATUS.VALID, updated:false}
}