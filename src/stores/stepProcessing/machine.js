import { STATUS } from "../Constants";

export const machineSteps = ({data, objectTypes, context, memo, solver, module, urdf}) => {
    return {steps:[], memo:{}, status:STATUS.VALID, updated:false}
}