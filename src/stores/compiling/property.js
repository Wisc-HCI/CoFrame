import { STATUS } from "../Constants";

export const propertyCompiler = ({data,properties}) => {
    return {newCompiled:{...properties,id:data.id}, memo:{}, status:STATUS.VALID}
}