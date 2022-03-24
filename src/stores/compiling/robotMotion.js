import { STATUS } from "../Constants";

export const robotMotionCompiler = ({data, objectTypes, context, path, memo, solver, module, urdf}) => {
    return {compiled:{[path]:[]}, memo:{}, status:STATUS.VALID, updated:false}
}