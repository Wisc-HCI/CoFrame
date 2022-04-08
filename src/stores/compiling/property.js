import { STATUS } from "../Constants";
import { equals } from "."; 

export const propertyCompiler = ({data,properties,objectTypes}) => {
    // console.log('propertyCompiler',properties)
    let status = STATUS.VALID;
    const objectSpec = objectTypes[data.type];
    Object.keys(properties).forEach(propKey=>{
        // console.log('considering ',propKey);
        if ((equals(properties[propKey],{}) || properties[propKey]===null) && !objectSpec.properties[propKey].nullValid) {
            // console.log('invalid/undefined prop',{propKey,value:properties[propKey]});
            status = STATUS.FAILED
        } else if (objectSpec.properties[propKey].isList) {
            properties[propKey].forEach(entry=>{
                if (equals(entry,{}) || entry===null) {
                    // console.log('invalid/undefined prop',{propKey,value:properties[propKey]});
                    status = STATUS.FAILED
                } else if (entry.properties && entry.properties.status === STATUS.FAILED) {
                    status = STATUS.FAILED
                }
            })
        } else if (properties[propKey].properties && properties[propKey].properties.status === STATUS.FAILED) {
            // console.log('failed prop',{propKey,value:properties[propKey]});
            status = STATUS.FAILED
        }
    })
    return {...properties,status}
}