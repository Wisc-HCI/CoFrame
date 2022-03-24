import { STATUS } from "../Constants";

export const skillCompiler = ({data, objectTypes, context, path, memo, solver, module, urdf}) => {
    return {compiled:{[path]:[]}, memo:{}, status:STATUS.VALID, updated:false}
}