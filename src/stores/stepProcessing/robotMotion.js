import { STATUS } from "../Constants";

export const robotMotionSteps = ({data, objectTypes, context, path, memo, solver, module, urdf}) => {
    return {steps:{[path]:[]}, memo:{}, status:STATUS.VALID, updated:false}
}