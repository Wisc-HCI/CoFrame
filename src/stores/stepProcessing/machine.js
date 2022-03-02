import { STATUS } from "../Constants";

export const machineSteps = ({data, objectTypes, context, path, memo, solver, module, urdf}) => {
    return {steps:{[path]:[]}, memo:{}, status:STATUS.VALID, updated:false}
}