import { STATUS } from "../Constants";

export const nullCompiler = ({path}) => {
    return {compiled:{[path]:[]}, memo:{}, status:STATUS.VALID, updated:false}
}