import { STATUS } from "../Constants";

export const linkCompiler = ({path}) => {
    return {newCompiled:null, memo:{}, status:STATUS.VALID, updated:false}
}