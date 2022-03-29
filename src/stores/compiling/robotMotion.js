import { STATUS } from "../Constants";

export const robotMotionCompiler = ({data, objectTypes, context, path, memo, solver, module, urdf}) => {
    return {status:STATUS.VALID,steps:[]}
}