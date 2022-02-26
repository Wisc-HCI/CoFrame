import { STATUS } from "../Constants";

export const skillSteps = ({data, objectTypes, context, memo, solver, module, urdf}) => {
    return {steps:[], memo:{}, status:STATUS.VALID, updated:false}
}