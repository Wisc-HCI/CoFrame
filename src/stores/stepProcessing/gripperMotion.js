import { STATUS } from "../Constants";

export const gripperMotionSteps = ({data, objectTypes, context, path, memo, solver, module, urdf}) => {
    return {steps:{[path]:[]}, memo:{}, status:STATUS.VALID, updated:false}
}