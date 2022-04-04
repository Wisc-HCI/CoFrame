import { STATUS } from "../Constants";

export const propertyCompiler = ({data,properties}) => {
    // console.log('propertyCompiler',properties)
    return {...properties,status:STATUS.VALID}
}