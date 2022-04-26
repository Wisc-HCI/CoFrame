import { STATUS } from "../Constants";
import { equals } from "."; 

const checkChild = (child,nullValid) => {
    if ((equals(child,{})||child===null) && !nullValid) {
        return STATUS.FAILED
    } else if (child.properties) {
        return child.properties.status
    } else {
        return STATUS.VALID
    }
}

export const propertyCompiler = ({data,properties,objectTypes}) => {
    // console.log('propertyCompiler',properties)
    let status = STATUS.VALID;
    const objectSpec = objectTypes[data.type];
    Object.keys(properties).forEach(propKey=>{
        // console.log('considering ',propKey);
        if (objectSpec.properties[propKey].isList) {
            properties[propKey].forEach(entry=>{
                let innerStatus = checkChild(entry,objectSpec.properties[propKey].nullValid);
                if (innerStatus === STATUS.FAILED || innerStatus === STATUS.WARN && status !== STATUS.FAILED) {
                    // Status is failed/warned, so the parent is also.
                    status = innerStatus;
                };
            })
        } else {
            let innerStatus = checkChild(properties[propKey],objectSpec.properties[propKey].nullValid);
            if (innerStatus === STATUS.FAILED || innerStatus === STATUS.WARN && status !== STATUS.FAILED) {
                // Status is failed/warned, so the parent is also.
                status = innerStatus;
            };
        }
    })
    return {...properties,status}
}