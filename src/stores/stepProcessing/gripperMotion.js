import { STATUS } from "../Constants";

export const gripperMotionSteps = ({data, objectTypes, context, memo, solver, module, urdf}) => {
    return {steps:[], memo:{}, status:STATUS.VALID, updated:false}
}