import { STATUS } from "../Constants";

export const propertyCompiler = ({data,properties,objectTypes}) => {
    // console.log('propertyCompiler',properties)
    let status = STATUS.VALID;
    const objectSpec = objectTypes[data.type];
    Object.keys(properties).forEach(propKey=>{
        if (!properties[propKey].id && !objectSpec.properties[propKey].nullValid) {
            status = STATUS.FAILED
        } else if (properties[propKey].properties && properties[propKey].properties.status === STATUS.FAILED) {
            status = STATUS.FAILED
        }
    })
    return {...properties,status}
}