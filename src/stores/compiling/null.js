import { STATUS } from "../Constants";

export const nullCompiler = () => {
    return {newCompiled:null, memo:{}, status:STATUS.VALID, updated:false}
}